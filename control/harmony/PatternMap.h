/**
 * @file PatternMap.h
 * @author Joseph Larralde
 * @date 14/08/2018
 * @brief map an incoming float on a scale defined by an interval pattern
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

#ifndef _JL_PATTERNMAP_H_
#define _JL_PATTERNMAP_H_

#define JL_MAX_PATTERN_MAP_LENGTH 64

#include <math.h>
#include "../../jl.h"

namespace jl {

template <typename T>
class PatternMap {
private:
  unsigned int pattern[JL_MAX_PATTERN_MAP_LENGTH];
  unsigned int patternSize;
  unsigned int length;
  float factor;
  T root;

  // tmp calculus variables
  T epsilon;
  T relRoot;
  T rootDiff;
  T relPosition;
  int lowerDegree;
  int upperDegree;
  T ratio;
  T scaled;

  // 0 : factor == 0, 1 : factor == 1, 2 : 0 < factor < 1
  unsigned int factorState;

public:
  PatternMap() :
  patternSize(1), length(1), factor(1), root(0),
  epsilon(1e-9), relRoot(0), rootDiff(0), relPosition(0),
  lowerDegree(0), upperDegree(1),
  ratio(1), scaled(0),
  factorState(1) {
    pattern[0] = 1;
  }

  ~PatternMap() {}

  void setPattern(unsigned int *pat, unsigned int len) {
    patternSize = 0;
    length = static_cast<unsigned int>(fmin(len, JL_MAX_PATTERN_MAP_LENGTH));

    for (unsigned int i = 0; i < length; ++i) {
      patternSize += JL_MAX(pat[i], 1);
      pattern[i] = JL_MAX(pat[i], 1);
    }

    if (length == 0) {
      patternSize = 1;
      length = 1;
      pattern[0] = 1;
    }
  }

  void setFactor(float f) {
    // factor interpolation feels more linear this way
    factor = pow(1 - (JL_CLIP(f, 0, 1)), 2);

    if (fabs(factor - 1) < epsilon) {
      factorState = 1;
    } else if (fabs(factor) < epsilon) {
      factorState = 0;
    } else {
      factorState = 2;
    }
  }

  void setRoot(T r) {
    root = r;
  }

  // some special cases are optimized :
  // - if factor == 1 : linear, pass value through
  // - if factor == 0 : don't compute curve, simply output closest value

  T process(T in) {
    if (factorState == 1) return in;

    relRoot = root;
    rootDiff = in - relRoot;


    // while (fabs(rootDiff) > patternSize) {
    while (rootDiff > patternSize) {
      relRoot += (rootDiff > 0 ? 1 : -1) * patternSize;
      rootDiff = in - relRoot;
    }

    // if (rootDiff < 0) {
    while (rootDiff < 0) {
      relRoot -= patternSize;
      rootDiff = in - relRoot;
    }

    relPosition = in - relRoot;
    lowerDegree = 0;
    upperDegree = pattern[0];

    for (unsigned int i = 0; i < length; ++i) {
      if (relPosition - upperDegree < 0) {
        break;
      }

      lowerDegree += pattern[i];
      upperDegree += pattern[(i + 1) % length];
    }

    // now scale da shit

    ratio = (relPosition - lowerDegree) / (upperDegree - lowerDegree);

    if (ratio < 0.5) {
      if (factorState == 0) {
        return relRoot + lowerDegree;
      }

      ratio *= 2;
      scaled = 1 - pow(1 - ratio, factor);
      scaled *= 0.5;
    } else {
      if (factorState == 0) {
        return relRoot + upperDegree;
      }

      ratio = (ratio - 0.5) * 2;
      scaled = pow(ratio, factor);
      scaled = (scaled * 0.5) + 0.5;
    }

    scaled *= (upperDegree - lowerDegree);
    return relRoot + lowerDegree + scaled;
  }
};

} /* end namespace jl */

#endif /* _JL_PATTERNMAP_H_ */