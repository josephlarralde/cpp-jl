/**
 * @file wavetable.h
 * @author Joseph Larralde
 * @date 07/06/2018
 * @brief set of interpolation functions and wavetable generators
 *
 * @copyright
 * Copyright (C) 2018 by Joseph Larralde.
 * All rights reserved.
 *
 * License (BSD 3-clause)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _JL_WAVETABLE_H_
#define _JL_WAVETABLE_H_

#include <cmath>
#include <type_traits>
#include <concepts>
#include "../../jl.h"

namespace jl {

//============================ INTERPOLATORS =================================//

//------------------------------- LINEAR -------------------------------------//

// p is minimum the 1st sample of a buffer
// and maximum the (n - 1)th sample of the same buffer
// (or loop over buffer length ?)

template <typename SI, typename SO, typename L, typename F>
// too beta for C++20 compilers ... :(
// template <
//   std::floating_point SI,
//   std::floating_point SO,
//   std::integral L,
//   std::floating_point F
// >
SO interpolateLinear(SI* buf, L index, F frac) {
  SI* p = buf + index;
  return static_cast<SO>((1.0f - frac) * (*p) + frac * (*(p + 1)));
}

// static inline float interpolateLinear(sample *buf, unsigned int index, float frac) {
//   sample *p = buf + index;

//   return (1. - frac) * (*p) + frac * (*(p + 1));
// }

template <typename SI, typename SO, typename L, typename F, typename I>
// template <
//   std::floating_point SI,
//   std::floating_point SO,
//   std::integral L,
//   std::floating_point F,
//   std::integral I
// >
void interpolateLinearStride(SI* buf, SO* res, L index, F frac, I stride) {
  SI* p = buf + index;

  for (I i = 0; i < stride; ++i) {
    *(res + i) = static_cast<SO>((1. - frac) * (*(p + i))
                                + frac * (*(p + i + stride)));
  }
}

// static inline void interpolateLinearStride(sample *buf, float *res, unsigned int index, float frac, unsigned int size) {
//   sample *p = buf + index;

//   for (unsigned int i = 0; i < size; ++i) {
//     *(res + i) = (1. - frac) * (*(p + i)) + frac * (*(p + i + size));
//   }
// }

//--------------------------------- CUBIC ------------------------------------//

// this is a cubic interpolation, mostly inspired from pure data's tabread4 object
// see : https://www.mail-archive.com/pd-list@iem.at/msg19006.html
// and updated link : http://msp.ucsd.edu/techniques/latest/book-html/node31.html

// dig this :
// http://en.dsplib.org/content/resampling_lagrange_ex/resampling_lagrange_ex.html#r2
// ==> might help to avoid aliasing

// also : understand the pink elephant paper and add some sinc based algorithms
// http://yehar.com/blog/wp-content/uploads/2009/08/deip.pdf

template <typename SI, typename SO, typename L, typename F>
// template <
//   std::floating_point SI,
//   std::floating_point SO,
//   std::integral L,
//   std::floating_point F
// >
SO interpolateCubic(SI* buf, L size, L index, F frac, bool cyclic = false) {
  SI* p = buf + index;
  SO a, b, c, d, cminusb;

  // if cyclic, wrap around buffer values, otherwise zero-pad

  a = (index <= 0)
    ? (cyclic ? *(buf + size + index - 1) : 0)
    : *(p - 1);

  b = *(p);

  c = (index > size - 2)
    ? (cyclic ? *(buf + index - size + 1) : 0)
    : *(p + 1);

  d = (index > size - 3)
    ? (cyclic ? *(buf + index - size + 2) : 0)
    : *(p + 2);

  cminusb = c - b;

  return b + frac * (
    cminusb - 0.1666667f * (1. - frac) *
    ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
  );

}

// static inline float interpolateCubic(sample *buf, unsigned int size,
//                                      unsigned int index, float frac,
//                                      bool cyclic = false) {
//   sample *p = buf + index;
//   sample a, b, c, d, cminusb;

//   // if cyclic, wrap around buffer values, otherwise zero-pad

//   a = (index <= 0)
//     ? (cyclic ? *(buf + size + index - 1) : 0)
//     : *(p - 1);

//   b = *(p);

//   c = (index > size - 2)
//     ? (cyclic ? *(buf + index - size + 1) : 0)
//     : *(p + 1);

//   d = (index > size - 3)
//     ? (cyclic ? *(buf + index - size + 2) : 0)
//     : *(p + 2);

//   cminusb = c - b;

//   return b + frac * (
//     cminusb - 0.1666667f * (1. - frac) *
//     ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
//   );
// }

// the same with a stride (for interleaved multi-channel data) ...

template <typename SI, typename SO, typename L, typename F, typename I>
// template <
//   std::floating_point SI,
//   std::floating_point SO,
//   std::integral L,
//   std::floating_point F,
//   std::integral I
// >
void interpolateCubicStride(SI* buf, SO* res, L size, L index, F frac,
                            I stride, bool cyclic = false) {
  SI* p = buf + index * stride;
  SO a, b, c, d, cminusb;

  for (I i = 0; i < stride; ++i) {
    a = (index <= 0)
      ? (cyclic ? *(buf + stride * (size + index - 1)) : 0)
      : *(p + i - stride);

    b = *(p + i);

    c = (index > size - 2)
      ? (cyclic ? *(buf + stride * (index - size + 1)) : 0)
      : *(p + i + stride);

    d = (index > size - 3)
      ? (cyclic ? *(buf + stride * (index - size + 2)) : 0)
      : *(p + i + 2 * stride);

    cminusb = c - b;

    *(res + i) = b + frac * (
      cminusb - 0.1666667f * (1. - frac) *
      ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
    );
  }

}

// static inline void interpolateCubicStride(sample *buf, sample *res, unsigned int size,
//                                           unsigned int index, float frac,
//                                           unsigned int stride, bool cyclic = false) {
//   sample *p = buf + index * stride;
//   sample a, b, c, d, cminusb;

//   for (unsigned int i = 0; i < stride; ++i) {
//     a = (index <= 0)
//       ? (cyclic ? *(buf + stride * (size + index - 1)) : 0)
//       : *(p + i - stride);

//     b = *(p + i);

//     c = (index > size - 2)
//       ? (cyclic ? *(buf + stride * (index - size + 1)) : 0)
//       : *(p + i + stride);

//     d = (index > size - 3)
//       ? (cyclic ? *(buf + stride * (index - size + 2)) : 0)
//       : *(p + i + 2 * stride);

//     cminusb = c - b;

//     *(res + i) = b + frac * (
//       cminusb - 0.1666667f * (1. - frac) *
//       ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
//     );
//   }
// }

// and the same with a second stride to allow reading into arrays of structs
// containing floats or doubles
// we use char * because it is of size 1 byte, which allows us to set the stride size
// to a buffer's item size in bytes (sampleSize), obtained with the sizeof operator

template <typename SO, typename L, typename F, typename I>
// template <
//   std::floating_point SO,
//   std::integral L,
//   std::floating_point F,
//   std::integral I
// >
void interpolateCubicStrideBytes(char* buf, SO* res, L size, L index, F frac,
                                 I stride, I sampleSize, bool cyclic = false) {
  I strideSize = stride * sampleSize;
  char* charBuf = static_cast<char*>(buf);
  char* p = charBuf + index * strideSize; // because char has a 1 byte size
  SO a, b, c, d, cminusb;

  for (I i = 0; i < stride; ++i) {
    a = (index <= 0)
      ? (cyclic ? *reinterpret_cast<SO*>(charBuf + strideSize * (size + index - 1)) : 0)
      : *reinterpret_cast<SO*>(p + i - sampleSize);

    b = *reinterpret_cast<SO*>(p + i);

    c = (index > size - 2)
      ? (cyclic ? *reinterpret_cast<SO*>(charBuf + strideSize * (index - size + 1)) : 0)
      : *reinterpret_cast<SO*>(p + i + sampleSize);

    d = (index > size - 3)
      ? (cyclic ? *reinterpret_cast<SO*>(charBuf + strideSize * (index - size + 2)) : 0)
      : *reinterpret_cast<SO*>(p + i + 2 * sampleSize);

    cminusb = c - b;

    *(res + i) = b + frac * (
      cminusb - 0.1666667f * (1. - frac) *
      ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
    );
  }
}

// static inline void interpolateCubicStrideBytes(char *buf, sample *res, unsigned int size,
//                                                unsigned int index, float frac,
//                                                unsigned int stride, unsigned int sampleSize,
//                                                bool cyclic = false) {
//   unsigned int strideSize = stride * sampleSize;
//   char *charBuf = static_cast<char *>(buf);
//   char *p = charBuf + index * strideSize; // because char has a 1 byte size
//   sample a, b, c, d, cminusb;

//   for (unsigned int i = 0; i < stride; ++i) {
//     a = (index <= 0)
//       ? (cyclic ? *reinterpret_cast<sample *>(charBuf + strideSize * (size + index - 1)) : 0)
//       : *reinterpret_cast<sample *>(p + i - sampleSize);

//     b = *reinterpret_cast<sample *>(p + i);

//     c = (index > size - 2)
//       ? (cyclic ? *reinterpret_cast<sample *>(charBuf + strideSize * (index - size + 1)) : 0)
//       : *reinterpret_cast<sample *>(p + i + sampleSize);

//     d = (index > size - 3)
//       ? (cyclic ? *reinterpret_cast<sample *>(charBuf + strideSize * (index - size + 2)) : 0)
//       : *reinterpret_cast<sample *>(p + i + 2 * sampleSize);

//     cminusb = c - b;

//     *(res + i) = b + frac * (
//       cminusb - 0.1666667f * (1. - frac) *
//       ((d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b))
//     );
//   }
// }

//============================== GENERATORS ==================================//

static inline void generateSine(sample *p, unsigned int size) {
  for (unsigned int i = 0; i < size; ++i) {
    *(p + i) = sin((float)i * 2.0f * M_PI / size);
  }
}

static inline void generateHann(sample *p, unsigned int size) {
  for (unsigned int i = 0; i < size; ++i) {
    *(p + i) = 0.5 * (1 - cos(((float)i * 2.0f * M_PI) / (size - 1)));
  }
}

// TODO : generateAATri, generateAASaw, generateAASqr (and also non-anti-aliased) ?

} /* end namespace jl */

#endif /* _JL_WAVETABLE_H_ */
