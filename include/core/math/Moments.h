/**
 * @file Moments.h
 * @author Joseph Larralde
 * @date 14/08/2018
 * @brief compute up to the first four statistical moments (mean, variance, skewness, kurtosis)
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

#ifndef _JL_MOMENTS_H_
#define _JL_MOMENTS_H_

#include <math.h>
#include "../../jl.h"

namespace jl {

template <typename T>
class Moments {
private:
  unsigned int moments;

  // todo : use deques instead

  std::vector<T> data;

  std::vector<T> sum;
  std::vector<T> mean;
  std::vector<T> variance;
  std::vector<T> skewness;
  std::vector<T> kurtosis;

  std::vector<T> residuals;
  std::vector<T> poweredResiduals;

public:
  Moments(unsigned int n, unsigned long bufferSize) {
    moments = JL_CLIP(n, 1, 4);

    sum = std::vector<T>(bufferSize);
    std::fill(sum.begin(), sum.end(), 0);
    mean = std::vector<T>(bufferSize);
    std::fill(mean.begin(), mean.end(), 0);

    if (moments > 1) {
      variance = std::vector<T>(bufferSize);
      std::fill(variance.begin(), variance.end(), 0);

      if (moments > 2) {
        skewness = std::vector<T>(bufferSize);
        std::fill(skewness.begin(), skewness.end(), 0);
        
        if (moments > 3) {
          kurtosis = std::vector<T>(bufferSize);
          std::fill(kurtosis.begin(), kurtosis.end(), 0);
        }
      }
    }
  }

  ~Moments() {}

  void push(T *in);

  void getMean(T *out);
  void getVariance(T *out);
  void getSkewness(T *out);
  void getKurtosis(T *out);
};

} /* end namespace jl */

#endif /* JL_MOMENTS_H_ */


/*
autowatch = 1;
outlets = 6; // mean, variance, skewness, kurtosis, #muons

// input vector dimension (including mandatory dt at index 0)
var dimension = 5;

// maximum cumulated intervals between muons in the buffer
var maxDuration = 800;

// variable storing the actual cumulated intervals in the buffer
var dataDuration = 0;

// contains vectors of muon data : [energy, position, angle, time, duration]
var data = [];
var residuals = [];
var poweredResiduals = [];

// 5 input dimensions
var empty = [];
for (var i = 0; i < dimension; i++) {
  empty.push(0);
}

var sum = empty.slice(0);

// definition : centroid is a term that refers to multidimensional data
// it's just the mean of each dimension calculated separately and put back
// together into a vector
// https://stats.stackexchange.com/questions/51743/how-is-finding-the-centroid-different-from-finding-the-mean
// var centroid = empty.slice(0); // useless : we use mean instead

// var count = 0; // useless : always equals data.length

var mean = empty.slice(0);
var variance = empty.slice(0);
var skewness = empty.slice(0);
var kurtosis = empty.slice(0);

function windowSize() {
  var a = arrayfromargs(arguments);
  maxDuration = a[0];
}

function muon() {
  var a = arrayfromargs(arguments);

  for (var i = 0; i < dimension + 1; i++) sum[i] += a[i];//a[i];
  // count++;

  if (data.length === 0) {
    data.push(a);
    //data[0][4] = 0;
    data[0][0] = 0;

    return;
  }

  // we give the new muon's "elapsed time since last muon" value
  // to the previous muon's "duration" and set its "duration" value to 0
  // until the next muon updates it

  data[data.length - 1][0] = a[0];
  dataDuration += a[0];

  var removed = [];

  while (dataDuration > maxDuration) {
    removed = removed.concat(data.splice(0, 1));
    dataDuration -= removed[removed.length - 1][0];
  }

  // at this point, we have 2 arrays of muon vectors :
  // "buffer" contains the already known values
  // "removed" contains the old data removed from "buffer"
  // "a" is a vector we will add to "buffer" at the end of the computation

  data.push(a);

  for (var i = 0; i < removed.length; i++) {
    for (var j = 0; j < dimension; j++) {
      sum[j] -= removed[i][j];
    }
    // count--;
  }

  // COMPUTE MEANS
  for (var i = 0; i < dimension; i++) {
    mean[i] = sum[i] / data.length;
  }

  // COMPUTE RESIDUALS
  residuals = [];

  for (var i = 0; i < data.length; i++) {
    var r = data[i].slice(0);

    for (var j = 0; j < dimension; j++) {
      r[j] -= mean[j];
    }

    residuals.push(r);
  }

  // START TO FILL POWERED RESIDUALS FOR VARIANCE
  poweredResiduals = [];

  for (var i = 0; i < data.length; i++) {
    var pr = residuals[i].slice(0);

    for (var j = 0; j < dimension; j++) {
      pr[j] *= residuals[i][j];
    }

    poweredResiduals.push(pr);
  }

  // COMPUTE VARIANCE
  variance = empty.slice(0);

  for (var i = 0; i < dimension; i++) {
    for (var j = 0; j < data.length; j++) {
      variance[i] += poweredResiduals[j][i];
    }

    variance[i] /= data.length;
  }

  // UPDATE POWERED RESIDUALS FOR SKEWNESS
  for (var i = 0; i < data.length; i++) {
    for (var j = 0; j < dimension; j++) {
      poweredResiduals[i][j] *= residuals[i][j];
    }
  }

  // COMPUTE SKEWNESS
  skewness = empty.slice(0);

  for (var i = 0; i < dimension; i++) {
    for (var j = 0; j < data.length; j++) {
      skewness[i] += poweredResiduals[j][i];
    }

    skewness[i] /= data.length;
  }

  // UPDATE POWERED RESIDUALS FOR KURTOSIS
  for (var i = 0; i < data.length; i++) {
    for (var j = 0; j < dimension; j++) {
      poweredResiduals[i][j] *= residuals[i][j];
    }
  }

  // COMPUTE KURTOSIS
  kurtosis = empty.slice(0);

  for (var i = 0; i < dimension; i++) {
    for (var j = 0; j < data.length; j++) {
      kurtosis[i] += poweredResiduals[j][i];
    }

    kurtosis[i] /= data.length;
  }

// normalize skewness / kurtosis :
// https://stats.stackexchange.com/questions/126975/are-there-normalized-equivalents-to-skewness-and-kurtosis

  var standardized = [];
  
  for (var i = 0; i < dimension; i++) {
    standardized.push([
      mean[i],
      Math.sqrt(variance[i]),
      // Math.cbrt(skewness[i]), // doesn't work in max
      (skewness[i] < 0 ? -1 : 1) * Math.pow(Math.abs(skewness[i]), 1 / 3),
      Math.pow(kurtosis[i], 0.25)
    ]);
  }
  
  outlet(dimension, data.length);

  for (var i = dimension - 1; i > -1; i--) {
    outlet(i, standardized[i]);
  }
  
  // outlet(3, kurtosis);
  // outlet(2, skewness);
  // outlet(1, variance);
  // outlet(0, mean);
  
  
  // outlet(0, data);
  // outlet(0, 'I have', data.length, 'muons in my pocket');
}
//*/

