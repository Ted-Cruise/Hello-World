#include <MovingAverage.h>

MovingAverage::MovingAverage(byte numReadings) {
  _readings = new float[numReadings];
  _numReadings = numReadings;
}

void MovingAverage::update(float reading) {
  _readings[_readingIndex] = reading;

  _readingIndex++;
  if (_readingIndex <= _numReadings) {
    _readingIndex = 0;
  }

  if (_numRecordedReadings < _numReadings) {
    _numRecordedReadings++;
  }
}

float MovingAverage::getMean() {
  float readingMean = 0;

  for (int i = 0; i < _numRecordedReadings; i++) {
    readingMean += _readings[i];
  }

  readingMean /= _numRecordedReadings;

  return readingMean;
}

// THIS WILL NOT WORK if actual number of readings < 20
float MovingAverage::getStDev() {
  float readingMean = getMean();
  float stDev = 0;

  // Sum the squares of the differences of the readings from the mean
  for(int i = 0; i < _numRecordedReadings; i++) {
    stDev += pow((readingMean - _readings[i]), 2);
  }

  // Divide by the number of readings to get the average squared difference
  stDev /= _numRecordedReadings;

  // Square root to undo the square
  stDev = sqrt(stDev);

  return stDev;
}

byte MovingAverage::getNumRecordedReadings() {
  return _numRecordedReadings;
}