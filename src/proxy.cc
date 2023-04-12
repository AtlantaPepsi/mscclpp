#include "alloc.h"
#include "checks.h"
#include "comm.h"
#include "debug.h"
#include "ib.h"
#include "socket.h"

#include <emmintrin.h>
#include <map>
#include <sys/syscall.h>
#include <thread>

#define MSCCLPP_PROXY_RUN_STATE_CHECK_PERIOD 100

#define PROXYCUDACHECK(cmd)                                                                                            \
  do {                                                                                                                 \
    cudaError_t err = cmd;                                                                                             \
    if (err != cudaSuccess) {                                                                                          \
      WARN("CUDA error from proxy: %s", cudaGetErrorString(err));                                                      \
      return NULL;                                                                                                     \
    }                                                                                                                  \
  } while (false)

#define PROXYMSCCLPPCHECK(call)                                                                                        \
  do {                                                                                                                 \
    mscclppResult_t res = call;                                                                                        \
    if (res != mscclppSuccess && res != mscclppInProgress) {                                                           \
      /* Print the back trace*/                                                                                        \
      if (mscclppDebugNoWarn == 0)                                                                                     \
        INFO(MSCCLPP_ALL, "%s:%d -> %d", __FILE__, __LINE__, res);                                                     \
      return NULL;                                                                                                     \
    }                                                                                                                  \
  } while (0);

struct proxyArgs
{
  struct mscclppComm* comm;
  struct mscclppProxyState* proxyState;
};

mscclppResult_t mscclppProxyFifo::create()
{
  MSCCLPPCHECK(mscclppCudaCalloc(&this->fifoHead, 1));
#if defined(MSCCLPP_USE_GDRCOPY)
  MSCCLPPCHECK(mscclppGdrCudaCalloc(&this->triggerFifo, &this->triggerFifoDev, MSCCLPP_PROXY_FIFO_SIZE,
                                    &this->triggerFifoDesc));
  MSCCLPPCHECK(
    mscclppGdrCudaCalloc(&this->fifoTailDevHostPtr, &this->fifoTailDev, 1, &this->fifoTailDesc));
#else
  MSCCLPPCHECK(mscclppCudaHostCalloc(&this->triggerFifo, MSCCLPP_PROXY_FIFO_SIZE));
  MSCCLPPCHECK(mscclppCudaCalloc(&this->fifoTailDev, 1));
#endif
  CUDACHECK(cudaStreamCreateWithFlags(&this->stream, cudaStreamNonBlocking));
  this->fifoTailHost = 0;
  return mscclppSuccess;
}

mscclppResult_t mscclppProxyFifo::destroy()
{
  MSCCLPPCHECK(mscclppCudaFree(this->fifoHead));
#if defined(MSCCLPP_USE_GDRCOPY)
  MSCCLPPCHECK(mscclppGdrCudaFree(this->triggerFifoDesc));
  MSCCLPPCHECK(mscclppGdrCudaFree(this->fifoTailDesc));
#else
  MSCCLPPCHECK(mscclppCudaHostFree(this->triggerFifo));
  MSCCLPPCHECK(mscclppCudaFree(this->fifoTailDev));
#endif
  CUDACHECK(cudaStreamDestroy(this->stream));
  return mscclppSuccess;
}

// return true if the trigger is valid
mscclppResult_t mscclppProxyFifo::poll(mscclppTrigger* trigger)
{
  __m128i xmm0 = _mm_load_si128((__m128i*)&this->triggerFifo[this->fifoTailHost % MSCCLPP_PROXY_FIFO_SIZE]);
  _mm_store_si128((__m128i*)trigger, xmm0);
  return mscclppSuccess;
}

mscclppResult_t mscclppProxyFifo::pop()
{
  *(volatile uint64_t*)(&this->triggerFifo[this->fifoTailHost % MSCCLPP_PROXY_FIFO_SIZE]) = 0;
  (this->fifoTailHost)++;
  return mscclppSuccess;
}

mscclppResult_t mscclppProxyFifo::flushTail(bool sync)
{
  // Flush the tail to device memory. This is either triggered every MSCCLPP_PROXY_FIFO_FLUSH_COUNTER to make sure
  // that the fifo can make progress even if there is no request mscclppSync. However, mscclppSync type is for flush
  // request.
#if defined(MSCCLPP_USE_GDRCOPY)
  *(volatile uint64_t*)(this->fifoTailDevHostPtr) = this->fifoTailHost;
#else
  CUDACHECK(
    cudaMemcpyAsync(this->fifoTailDev, &(this->fifoTailHost), sizeof(uint64_t), cudaMemcpyHostToDevice, this->stream));
  if (sync) {
    CUDACHECK(cudaStreamSynchronize(this->stream));
  }
#endif
  return mscclppSuccess;
}

static void processTrigger(const mscclppTrigger trigger, mscclppConn* conn)
{
  // Iterate over what send is needed
  if (trigger.fields.type & mscclppData) {
    conn->hostConn->put(trigger.fields.dstDataOffset, trigger.fields.srcDataOffset, trigger.fields.dataSize);
  }

  if (trigger.fields.type & mscclppFlag) {
    conn->hostConn->signal();
  }

  // Wait for completion
  if (trigger.fields.type & mscclppSync) {
    conn->hostConn->flush();
  }
}


void* mscclppProxyService(void* _args)
{
  struct proxyArgs* args = (struct proxyArgs*)_args;
  struct mscclppComm* comm = args->comm;
  struct mscclppProxyState* proxyState = args->proxyState;
  free(_args); // allocated in mscclppProxyCreate

  // from this point on, proxy thread will stay close to the device
  PROXYMSCCLPPCHECK(numaBind(comm->devNumaNode));

  struct mscclppProxyFifo* fifo = &proxyState->fifo;
  volatile mscclppProxyRunState_t* run = &proxyState->run;
  mscclppTrigger trigger;

  int runCnt = MSCCLPP_PROXY_RUN_STATE_CHECK_PERIOD;
  uint64_t flushCnt = 0;
  for (;;) {
    if (runCnt-- == 0) {
      runCnt = MSCCLPP_PROXY_RUN_STATE_CHECK_PERIOD;
      if (*run != MSCCLPP_PROXY_RUN_STATE_RUNNING) {
        break;
      }
    }
    // Poll to see if we are ready to send anything
    PROXYMSCCLPPCHECK(fifo->poll(&trigger));
    if (trigger.value[0] == 0) {
      continue; // there is one in progreess
    }
    
    mscclppConn* conn = &comm->conns[trigger.fields.connId];
    processTrigger(trigger, conn);

    // Send completion: reset only the high 64 bits
    PROXYMSCCLPPCHECK(fifo->pop());
    // Flush the tail to device memory. This is either triggered every MSCCLPP_PROXY_FIFO_FLUSH_COUNTER to make sure
    // that the fifo can make progress even if there is no request mscclppSync. However, mscclppSync type is for flush
    // request.
    if (((++flushCnt % MSCCLPP_PROXY_FIFO_FLUSH_COUNTER) == 0) || (trigger.fields.type & mscclppSync)) {
      PROXYMSCCLPPCHECK(fifo->flushTail());
    }
  }

  // make sure the tail is flushed before we shut the proxy
  PROXYMSCCLPPCHECK(fifo->flushTail(/*sync=*/true));
  bool isP2pProxy = (proxyState->ibContext == nullptr);
  if (isP2pProxy) {
    cudaStream_t p2pStream = proxyState->p2pStream;
    PROXYCUDACHECK(cudaStreamSynchronize(p2pStream));
  }
  *run = MSCCLPP_PROXY_RUN_STATE_IDLE;
  return NULL;
}

mscclppResult_t mscclppProxyCreate(struct mscclppComm* comm)
{
  for (int i = 0; i < MSCCLPP_PROXY_MAX_NUM; ++i) {
    struct mscclppProxyState* proxyState = comm->proxyState[i];
    if (proxyState == NULL)
      break;

    struct proxyArgs* args;
    MSCCLPPCHECK(mscclppCalloc(&args, 1));
    args->comm = comm;
    args->proxyState = proxyState;

    proxyState->run = MSCCLPP_PROXY_RUN_STATE_RUNNING;
    pthread_create(&proxyState->thread, NULL, mscclppProxyService, args);
    if (proxyState->transportType == mscclppTransportP2P) {
      mscclppSetThreadName(proxyState->thread, "MSCCLPP Service P2P - %02d", comm->cudaDev);
    } else if (proxyState->transportType == mscclppTransportIB) {
      mscclppSetThreadName(proxyState->thread, "MSCCLPP Service IB - %02d", i);
    }
  }
  return mscclppSuccess;
}

mscclppResult_t mscclppProxyDestroy(struct mscclppComm* comm)
{
  for (int i = 0; i < MSCCLPP_PROXY_MAX_NUM; ++i) {
    struct mscclppProxyState* proxyState = comm->proxyState[i];
    if (proxyState == NULL)
      break;

    volatile int* run = (volatile int*)&proxyState->run;
    if (*run == MSCCLPP_PROXY_RUN_STATE_IDLE) {
      continue;
    }
    *run = MSCCLPP_PROXY_RUN_STATE_EXITING;
    while (*run == MSCCLPP_PROXY_RUN_STATE_EXITING && *comm->abortFlag == 0) {
      usleep(1000);
    }
  }
  return mscclppSuccess;
}
