/**
 * @file Average.h
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief simple running mean computer
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

// NB : this will be deprecated when the Moments class is operational because
// what the Average class does is just computing the first statistical moment of
// a fifo buffer

#ifndef _JL_AVERAGE_H_
#define _JL_AVERAGE_H_

#include <deque>

namespace jl {

template <typename T>
class Average {
private:
  unsigned long fifoSize;
  T zero;
  T sum;
  std::deque<T> fifo;

public:
  Average(unsigned long size = 1) : fifoSize(size) {
    zero = static_cast<T>(0);
    sum = zero;

    for (unsigned long i = 0; i < fifoSize; ++i) {
      fifo.push_back(zero);
    }
  }

  ~Average() {}

  void resize(unsigned long size) {
    unsigned long newSize = JL_MAX(size, 1);
    long delta = newSize - fifoSize;
    fifoSize = newSize;

    if (delta > 0) {
      for (long i = 0; i < delta; ++i) {
        fifo.push_front(zero);
      }
    } else if (delta < 0) {
      for (long i = 0; i < delta * -1; ++i) {
        fifo.pop_front();
      }
    }
  }

  T process(T in) {
    sum -= fifo.front();
    fifo.pop_front();

    sum += in;
    fifo.push_back(in);

    return sum / fifoSize;
  }
};

} /* end namespace jl */

#endif /* _JL_AVERAGE_H_ */