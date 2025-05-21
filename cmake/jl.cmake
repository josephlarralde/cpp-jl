# include(CMakeFindDependencyMacro)

set(CMAKE_INCLUDE_CURRENT_DIR 1)
# set(CMAKE_CXX_STANDARD 23)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(JL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(JL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "")

# find_package(Boost QUIET REQUIRED)
# find_package(Threads QUIET)
# find_package(fmt QUIET)

set(JL_SOURCES
  "${JL_SOURCE_DIR}/include/core/filters/Average.h"
  "${JL_SOURCE_DIR}/include/core/filters/Biquad.h"
  "${JL_SOURCE_DIR}/include/core/filters/OneEuro.h"
  "${JL_SOURCE_DIR}/include/core/filters/OnePole.h"
  "${JL_SOURCE_DIR}/include/core/filters/WeightedMean.h"
  "${JL_SOURCE_DIR}/include/core/math/KdTree.h"
  "${JL_SOURCE_DIR}/include/core/math/Moments.h"

  "${JL_SOURCE_DIR}/include/dsp/effects/distortions/Distort.h"
  "${JL_SOURCE_DIR}/include/dsp/effects/dynamics/Compress.h"
  "${JL_SOURCE_DIR}/include/dsp/effects/temporal/DelayLine.h"
  "${JL_SOURCE_DIR}/include/dsp/effects/temporal/Stut.h"
  "${JL_SOURCE_DIR}/include/dsp/sampler/Gbend.h"
  "${JL_SOURCE_DIR}/include/dsp/synthesis/Oscillator.h"
  "${JL_SOURCE_DIR}/include/dsp/utilities/FFT.h"
  "${JL_SOURCE_DIR}/include/dsp/utilities/Ramp.h"
  "${JL_SOURCE_DIR}/include/dsp/utilities/units.h"
  "${JL_SOURCE_DIR}/include/dsp/utilities/wavetable.h"

  "${JL_SOURCE_DIR}/include/control/harmony/PatternMap.h"
  "${JL_SOURCE_DIR}/include/control/harmony/Tonnetz.h"
  "${JL_SOURCE_DIR}/include/control/utilities/IntervalMap.h"
  "${JL_SOURCE_DIR}/include/control/utilities/routing.h"
)

add_library(jl_lib OBJECT
  "${JL_SOURCE_DIR}/include/dsp/effects/distortions/Distort.cpp"
  "${JL_SOURCE_DIR}/include/dsp/effects/dynamics/Compress.cpp"
  "${JL_SOURCE_DIR}/include/dsp/effects/temporal/DelayLine.cpp"
  "${JL_SOURCE_DIR}/include/dsp/effects/temporal/Stut.cpp"
  "${JL_SOURCE_DIR}/include/dsp/sampler/Gbend.cpp"
  "${JL_SOURCE_DIR}/include/dsp/synthesis/Oscillator.cpp"
)
