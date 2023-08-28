/**
 * @file FIR.h
 * @author Joseph Larralde
 * @date 28/07/2023
 * @brief finite (parametric) impulse response filter
 *
 * @copyright
 * Copyright (C) 2023 by Joseph Larralde.
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

#ifndef _JL_WEIGHTEDMEAN_H_
#define _JL_WEIGHTEDMEAN_H_

#include "../../jl.h"

namespace jl {

template <typename T>
class WeightedMean {
  std::size_t windowSize;
  std::deque<T> window;

  float weightFactor;
  float weightUnit; // 1.0f / windowSize : smallest weight when factor is 1
  float weightSum;
  std::vector<float> weights;

public:
  WeightedMean(std::size_t maxWinSize = 32) :
  weightFactor(1.0f) {
    window.resize(maxWinSize, static_cast<T>(0));
    weights.resize(maxWinSize, 0.0f);
    setWindowSize(maxWinSize);
    computeWeights();
  }

  void setWindowSize(std::size_t size) {
    windowSize = std::minimum(size, window.size());
    weightUnit = 1.0f / windowSize;
    computeWeights();
  }

  void setWeightFactor(float f) {
    weightFactor = f;
    computeWeights();
  }

  T process(T val) {
    window.push_front(val);
    window.pop_back();

    T res = static_cast<T>(0);

    for (std::size_t = 0; i < windowSize; ++i) {
      res += static_cast<T>(window[i] * weights[i]);
    }

    return res / static_cast<T>(weightSum);
  }

private:
  void computeWeights() {
    weightSum = 0.0f;
    for (std::size_t i = 0; i < windowSize; ++i) {
      weights[i] = std::pow((windowSize - i) * weightUnit, weightFactor);
      weightSum += weights[i];
    }
  }
};

} // end namespace jl

#endif /* _JL_WEIGHTEDMEAN_H_ */