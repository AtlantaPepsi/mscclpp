// Copyright (c) Microsoft Corporation.
// Licensed under the MIT license.

#ifndef MSCCLPP_POLL_DEVICE_HPP_
#define MSCCLPP_POLL_DEVICE_HPP_

#include "device.hpp"

#if defined(MSCCLPP_DEVICE_COMPILE)

#include <cstdint>

#if defined(NDEBUG)
#define __assert_fail(__assertion, __file, __line, __function) ;
#else  // !defined(NDEBUG)
#if defined(MSCCLPP_DEVICE_HIP)
extern "C" __device__ void __assert_fail(const char *__assertion, const char *__file, unsigned int __line,
                                         const char *__function);
#else   // !defined(MSCCLPP_DEVICE_HIP)
extern "C" __host__ __device__ void __assert_fail(const char *__assertion, const char *__file, unsigned int __line,
                                                  const char *__function) __THROW;
#endif  // !defined(MSCCLPP_DEVICE_HIP)
#endif  // NDEBUG

static constexpr uint64_t clocks_factor = 2106;

static void __device__
sleep_clocks (uint64_t clocks)
{
  if (threadIdx.x % 64 == 0)
  {
    uint64_t start = clock64 ();
    for (;;)
    {
      uint64_t now = clock64 ();
      uint64_t elapsed = (now > start
			  ? now - start
			  : now + (0xffffffffffffffff - start));
      if (elapsed >= clocks)
	return;
    }
  }
}

extern "C" void __device__
nsleep (int nsec)
{
  uint64_t clocks = clocks_factor * CLOCKS_PER_SEC * nsec / 1000000000;

  sleep_clocks (clocks);
}

// If a spin is stuck, print a warning and keep spinning.
#define POLL_MAYBE_JAILBREAK(__cond, __max_spin_cnt)                     \
  do {                                                                   \
    int64_t __spin_cnt = 0;                                              \
    while (__cond) {                                                     \
      if (__max_spin_cnt >= 0 && __spin_cnt++ == __max_spin_cnt) {       \
        __assert_fail(#__cond, __FILE__, __LINE__, __PRETTY_FUNCTION__); \
      }                                                                  \
    }                                                                    \
  } while (0);

// the as POLL_MAYBE_JAILBREAK except that __cond1 is checked before __cond2
// this is specially useful when __cond1 is faster to check
#define OR_POLL_MAYBE_JAILBREAK(__cond1, __cond2, __max_spin_cnt)                  \
  do {                                                                             \
    int64_t __spin_cnt = 0;                                                        \
    while (true) {                                                                 \
      if (!(__cond1)) {                                                            \
        break;                                                                     \
      } else if (!(__cond2)) {                                                     \
        break;                                                                     \
      }                                                                            \
      if (__max_spin_cnt >= 0 && __spin_cnt++ == __max_spin_cnt) {                 \
        __assert_fail(#__cond1 #__cond2, __FILE__, __LINE__, __PRETTY_FUNCTION__); \
      }                                                                            \
    }                                                                              \
  } while (0);

#endif  // defined(MSCCLPP_DEVICE_COMPILE)

#endif  // MSCCLPP_POLL_DEVICE_HPP_
