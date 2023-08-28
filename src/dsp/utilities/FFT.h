// TODO : implement FFT and iFFT

// first goal : write an omptimized partitioned convolution algorithm to allow
// for implementation of reverberation effects

// then eventually implement other spectral effects such as filters, gizmo
// (spectral domain pitch shift abstraction in Max), etc.

// this class should take incoming vectors au time-domain signal
// it should also have a window size (power of 2) and overlap (integer) arguments

class FFT {
  FFT(std::size_t windowSize = 1024) {
    //
  }

  ~FFT() {}
};