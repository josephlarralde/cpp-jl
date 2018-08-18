/**
 * @file Oscillator.cpp
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief oscillators
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

#include "Oscillator.h"
#include "../utilities/wavetable.h"

namespace jl {

void
WavetableOscillator::setSamplingRate(float f) {
  sr = f;
  msr = sr / 1000;
}

void
WavetableOscillator::setFrequency(float f) {
  // ?
}

void
WavetableOscillator::setPhase(float f) {
 // ?
}

// TODO : implement poly BLEPS
// http://metafunction.co.uk/all-about-digital-oscillators-part-2-blits-bleps/

// TODO : implement phase modulation (as in fm feedback)

void
WavetableOscillator::process(sample *in, sample *out, unsigned int blockSize) {
  float position;

  for (unsigned int i = 0; i < blockSize; ++i) {

    // TODO : do something to bring phase back to 0 when frequency is 0 and phase is not ?
    // could damage loudspeakers if keeping away from the HP's rest position ...

    // phase is a normalized value E [0;1[
    // dt (duration of an audio sample in seconds) = 1 / sr
    // p (duration of a period of frequency f) = 1 / f
    // for each sample, the phase must be incremented by (dt / p) <=> (f / sr)
    //
    phase += *in++ / sr;

    if (phase >= 1) {
      phase -= floor(phase);
    }

    // EVENTUALLY APPLY PHASE DISTORTION HERE :)

    position = phase * tableSize;

    unsigned long index = floor(position);
    float frac = position - index;

    // would using cyclic and change tableSize whenever needed smooth the signal
    // enough at its discontinuities to be a replacement for BLIT BLEPS etc ?
    *out++ = interpolateCubic(table, tableSize, index, frac, true);
  }
}

//============================================================================//
// PRIVATE :

void
WavetableOscillator::createTable() {
  switch (shape) {
    // case OscShapeSin:
    default: // for now always generate a sine period
      generateSine(table, tableSize);
      break;
  }
}

} /* end namespace jl */