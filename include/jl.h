/**
 * @file jl.h
 * @author Joseph Larralde
 * @date 01/08/2018
 * @brief main header of the jl library
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

#ifndef _JL_H_
#define _JL_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define JL_VERSION_MAJOR 0
#define JL_VERSION_MINOR 0
#define JL_VERSION_PATCH 1
#define JL_VERSION "0.0.1"

#define JL_MAX_BLOCK_SIZE 4096

#define JL_MIN(X, M) (X > M) ? M : X
#define JL_MAX(X, M) (X < M) ? M : X
#define JL_CLIP(X, A, B) (X < A) ? A : ((X > B) ? B : X)

namespace jl {
  typedef float sample;
  typedef float value;
}

#include "dsp/utilities/units.h"
#include "dsp/utilities/wavetable.h"
#include "dsp/utilities/Ramp.h"
#include "dsp/effects/dynamics/Compress.h"
#include "dsp/effects/temporal/Stut.h"
#include "dsp/sampler/Gbend.h"
#include "dsp/synthesis/Oscillator.h"

#endif /* _JL_H_ */