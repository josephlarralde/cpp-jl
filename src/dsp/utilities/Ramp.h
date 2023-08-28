/**
 * @file Ramp.h
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief all-purpose audio ramp generator
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

#ifndef _JL_RAMP_H_
#define _JL_RAMP_H_

namespace jl {

// super basic linear ramping class, working in sample units

// see https://dsp.stackexchange.com/questions/14754/equal-power-crossfade/49989#49989
// (a very detailed post from robert bristow-johnson about cross-fading)

// some say on music-dsp that half a cosine period is as fine as a linear ramp
// TODO : compare both kinds of fade and choose by ear

// the Gbend class could be rewritten using mostly ramps (for fades, interrupt and buffer index)
// TODO : create a speed factor driven version of the ramp (?)

template <typename TI, typename TO>
class Ramp {
private:
  TO value;
  TO target;
  TO *block;//[JL_MAX_BLOCK_SIZE];
  // std::vector<TO> block;
  unsigned int bSize;
  bool ramping;
  TO increment;
  unsigned long sRemaining;

public:
  Ramp(unsigned int blockSize = 64) : bSize(blockSize) {
    block = new TO[bSize];
  }

  ~Ramp() {
    delete[] block;
  }

  bool isRamping() {
    return ramping;
  }

  void ramp(TI t, unsigned long samples = 0) {
    if (samples == 0) {
      ramping = false;
      value = static_cast<TO>(t);
    } else {
      target = static_cast<TO>(t);
      increment = (t - value) / samples;
      ramping = true;
      sRemaining = samples;
    }
  }

  TO *process(unsigned int blockSize) {
    if (blockSize != bSize) {
      delete[] block;
      bSize = blockSize;
      block = new TO[bSize];
    }

    TO *out = &(block[0]);

    for (unsigned int i = 0; i < blockSize; ++i) {
      if (ramping) {
        if (sRemaining == 0) {
          ramping = false;
          value = target;
          *out++ = value;
        } else {
          value += increment;
          *out++ = value;
          sRemaining--;
        }
      } else {
        *out++ = value;
      }
    }

    return &(block[0]);
  }
};

} /* end namespace jl */

#endif /* _JL_RAMP_H_ */
