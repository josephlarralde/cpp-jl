/**
 * @file units.h
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief audio related unit converters
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

#ifndef _JL_UNITS_H_
#define _JL_UNITS_H_

#include <cmath>
#include <cassert>
#include <limits>
#include "../../jl.h"

namespace jl {

// source :
// http://www.sengpielaudio.com/calculator-FactorRatioLevelDecibel.htm

// amplitude 1 <=> 0 dB

static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 required");
static_assert(std::numeric_limits<double>::is_iec559, "IEEE 754 required");

template <typename T>
T clipInfinity(T val) {
  if (val == -std::numeric_limits<T>::infinity()) {
    return std::numeric_limits<T>::lowest();
  } else if (val == std::numeric_limits<T>::infinity()) {
    return std::numeric_limits<T>::max();
  }

  return val;
}

template <typename T>
T dbtoa(T db) {
  T res = pow(10, clipInfinity<T>(db) / 20);
  return clipInfinity<T>(res);
}

template <typename T>
T atodb(T a) {
  T res = 20 * log10(clipInfinity<T>(a));
  return clipInfinity<T>(res);
}

} /* end namespace jl */

#endif /* _JL_UNITS_H_ */