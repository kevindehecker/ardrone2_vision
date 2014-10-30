#include "smoother.h"

  void Smoother::init (int width ) {
  	_kernelsize = width;	
	_rbuf.resize(_kernelsize+1); // strange, +1 seems needed for a vector of actual size???
	_rotater = 0;

  }


float Smoother::addSample(float sample) {
	//performs online smoothing filter

	if (isnan(sample)) // fixes nan, which forever destroy the output
		sample = 0;

	_rbuf.at(_rotater) = sample; // overwrite oldest sample in the roundtrip buffer
	_rotater = (_rotater+1) % (_kernelsize); //update pointer to buffer
	_runner = _runner + sample - _rbuf.at(_rotater); //add new sample, subtract the new oldest sample 

	return _runner /_kernelsize;

}
