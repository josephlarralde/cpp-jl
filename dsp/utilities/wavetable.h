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
#include "../../jl.h"

namespace jl {

//============================ INTERPOLATORS =================================//

//------------------------------- LINEAR -------------------------------------//

// p is minimum the 1sr sample of a buffer
// and maximum the (n - 1)th sample of the same buffer
// (or loop over buffer length ?)
static inline float interpolateLinear(sample *buf, unsigned int index, float frac) {
  float *p = buf + index;

  return (1. - frac) * (*p) + frac * (*(p + 1));
}

static inline void interpolateLinearStride(sample *buf, float *res, unsigned int index, float frac, unsigned int size) {
  float *p = buf + index;

  for (unsigned int i = 0; i < size; ++i) {
    *(res + i) = (1. - frac) * (*(p + i)) + frac * (*(p + i + size));
  }
}

//------------------------------- BICUBIC ------------------------------------//

// mostly inspired from pure data's tabread4 object
// TODO : understand how miller's algorithm works
// what kind of optimization is this ? is "cubic" an appropriate name ?
// dig this :
// http://en.dsplib.org/content/resampling_lagrange_ex/resampling_lagrange_ex.html#r2
// ==> might help to avoid aliasing

// also : understand the pink elephant paper and add some sinc based algorithms
// http://yehar.com/blog/wp-content/uploads/2009/08/deip.pdf

static inline float interpolateCubic(sample *buf, unsigned int size,
                                       unsigned int index, float frac,
                                       bool cyclic = false) {
  float *p = buf + index;
  float a, b, c, d, cminusb;

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

// the same with a stride ...

static inline void interpolateCubicStride(sample *buf, sample *res, unsigned int size,
                                            unsigned int index, float frac,
                                            unsigned int stride, bool cyclic = false) {
  float *p = buf + index * stride;
  float a, b, c, d, cminusb;

  for (unsigned int i = 0; i < stride; ++i) {
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
