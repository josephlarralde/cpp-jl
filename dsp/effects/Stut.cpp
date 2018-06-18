/**
 * @file Stut.cpp
 * @author Joseph Larralde
 * @date 09/06/2018
 * @brief pitch signal controlled stutter effect with integrated fade in / fade out system and start / stop messages
 *
 * Stutter effect featuring :
 *
 * It is intended to be simple enough to allow an easy integration into a variety
 * of environments
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

#include "../../jl.h"
#include "Stut.h"
#include "m_pd.h"

void
Stut::setSamplingRate(float f) {
  // need to resize the internal buffer here
  unsigned long nextBufferSamples = (unsigned long) ((f / 1000) * bufferDuration);

  if (nextBufferSamples != bufferSamples) {
    msr = f / 1000;
    bufferSamples = nextBufferSamples;

    delete [] buffer;
    buffer = new float[bufferSamples * channels];
  }
}

void
Stut::setSliceDuration(float f) {
  slice = JL_MAX(f, 1);
}

void
Stut::setSlices(long n) {
  nextLoops = n;
}

void
Stut::setFades(float f) {
  fadi = fado = JL_MAX(f, 1);
}

void
Stut::setFadeIn(float f) {
  fadi = JL_MAX(f, 1);
}

void
Stut::setFadeOut(float f) {
  fado = JL_MAX(f, 1);
}

void
Stut::setInterrupt(float f) {
  interrupt = JL_MAX(f, 1);
}

void
Stut::setRelease(float f) {
  release = JL_MAX(f, 1);
}

void
Stut::start() {
  if (!interrupting) {
    interrupting = true;
    interruptIndex = 0;
    realPlaying = true;

    if (!playing) {
      playing = true;
      // this allows us to interrupt either on input or on current buffer reading :
      loopCounter = -1;
    }
  }
}

void
Stut::stop() {
  if(!stopping) {
    stopping = true;
    realStopping = true;
  }
}

void
Stut::process(float **ins, float **outs, unsigned int blockSize) {
  long longBlockSize = (long)(blockSize);

  if (blk != longBlockSize) {
    blk = longBlockSize;
  }

  long ntmp = blk;
  float iVal = 1; // interrupt fade value (normalized)
  float rVal = 1; // release fade value (normalized)
  float fVal = 1; // actual fade (in or out) value (normalized)

  // what's that magic already ?
  if (stopping) {
    sStopLen = (index + sFado < sStopLen)
             ? (index + sFado)
             : sStopLen;
    stopping = false;
  }

  while (ntmp-- > 0) {
    int currentIndex = blk - ntmp - 1;

    //_______ FILL THE BUFFER IF NEEDED ____________________________________//

    if (recording) {
      if (recordIndex < bufferSamples) {
        for (unsigned int c = 0; c < channels; ++c) {
          buffer[recordIndex * channels + c] = ins[c][currentIndex];
        }

        recordIndex++;
      } else {
        recording = false;
      }
    }

    //_______ SET RELEASE FADE VALUE _______________________________________//

    if (releasing && releaseIndex < sRelease) {
      rVal = (float) (releaseIndex) / (sRelease - 1);
      releaseIndex++;
    } else {
      releasing = false;
      rVal = 1;
    }

    //_______ SET INTERRUPT FADE VALUE _____________________________________//

    if (interrupting) {
      if (interruptIndex < sInterrupt) {
        iVal = 1 - (float) (interruptIndex) / (sInterrupt - 1);
        interruptIndex++;
      }
    } else {
      iVal = 1;
    }

    //_______ THEN SET CURRENT FADE IN OR OUT VALUE ________________________//

    fVal = (index < sFadi)
         ? ((index < 0)
            ? 0
            : ((float) index / sFadi))
         : ((index > sStopLen - sFado)
            ? ((index > sStopLen)
                ? 0
                : ((float) (sStopLen - index) / sFado))
            : 1);

    //________ NOW COMPUTE THE ACTUAL AUDIO OUT ____________________________//

    // we read the actual sample whether it's just been recorded or not
    // it's the same thing as taking directly from the input (in principle)

    for (unsigned int c = 0; c < channels; ++c) {
      float *out = outs[c];

      if (!playing || (interrupting && loopCounter == -1)) { //  we were not playing and got triggered
        out[currentIndex] = ins[c][currentIndex] * rVal * iVal;
      } else if (index < (long) bufferSamples) {
        out[currentIndex] = buffer[index * channels + c] * iVal * fVal;
      } else {
        out[currentIndex] = 0;
      }
    }

    //_______ CHECK IF WE NEED TO LOOP OR START ____________________________//

    if (playing && !interrupting && (index > sStopLen || index < 0)) { // we're going too far ...
      if (realStopping) {
        realStopping = false;
        playing = false;
        realPlaying = false;
        releasing = true;
        releaseIndex = 0;
        endReachCallback(0); // manual end
      } else if (loops < 0 || loopCounter < loops - 1) {
        index = 0;
        loopCounter++;
        computeParameters();
        endReachCallback(2); // looping
      } else {
        playing = false;
        realPlaying = false;
        releasing = true;
        releaseIndex = 0;
        endReachCallback(1); // natural end
      }
    }

    if (playing) {
      index++;
    }

    //_________THIS_IS_THE_ONLY_GOOD_PLACE_TO_TRIG_SAMPLE_PLAYING___________//

    // AN EXCEPTION :
    // in case of end playing and retrig during the same signal vector
    // we use the variable realPlaying set to true by bang()
    // because playing might have been set to false in this loop

    if (!interrupting && realPlaying) {
      computeParameters();

      playing = true;
      realPlaying = false;
    }

    // MOST GENERAL CASE :

    if (interrupting && (interruptIndex >= sInterrupt)) { // interrupt is just finished
      interrupting = false;
      interruptIndex = 0;
      recording = true;
      recordIndex = 0;
      loopCounter = 0;
      index = 0;

      computeParameters();
    }
  }
}

void
Stut::endReachCallback(int endReachType) {
  // do nothing in base class
}

//============================================================================//

void
Stut::computeParameters() {
  loops = nextLoops;
  // MINIMUM SELECTION 1 SIGNAL BLOCK SIZE
  // TODO: CHECK IF MINIMUM 2 SAMPLES WOULD BE BETTER (ALLOWING 1 SAMPLE FADES)

  sSlice = (long) (slice * msr);
  sSlice = (sSlice > blk) ? sSlice : blk;
  sStopLen = sSlice;

  sFadi = (long) (fadi * msr);
  sFado = (long) (fado * msr);
  sInterrupt = (long) (interrupt * msr);
  sRelease = (long) (release * msr);

  if (sFadi + sFado >= sSlice) {
    if (sFadi < sSlice / 2) {
      sFado = sSlice - sFadi;
    } else if (sFado < sSlice / 2) {
      sFadi = sSlice - sFado;
    } else {
      sFadi = sFado = sSlice / 2;
    }
  }

  if (sInterrupt > sSlice - sFadi) {
    sInterrupt = sSlice - sFadi; // avoid clicking at the end if interrupt is longer than selection
    // not very clear, though ... why not using sSlice directly ?
  }
}
