#include <vector>

bool squareMatrixWillLoop(const std::vector<bool>& matrix,
                          int matrixDimension,
                          int inputIndex,
                          int outputIndex,
                          int originalOutputIndex = -1) {
  // assert(matrix.size() == matrixDimension * matrixDimension); // ?

  bool connected = true;

  if (originalOutputIndex == -1) {
    originalOutputIndex = outputIndex;
  } else {
    connected = matrix[inputIndex + outputIndex * matrixDimension];
  }

  // this will return directly if we try to connect input N to output N
  if (connected) {
    if (inputIndex == originalOutputIndex) {
      return true;
    }

    for (int i = 0; i < matrixDimension; ++i) {
      if (squareMatrixWillLoop(matrix,
                              matrixDimension,
                              i,
                              inputIndex,
                              originalOutputIndex)) {
        return true;
      }
    }
  }
  
  return false;
}