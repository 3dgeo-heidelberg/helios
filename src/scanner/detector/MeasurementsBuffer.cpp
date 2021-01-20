#include "MeasurementsBuffer.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
MeasurementsBuffer::MeasurementsBuffer(const MeasurementsBuffer &mb){
    for(size_t i = 0 ; i < MEASUREMENTS_BUFFER_SIZE ; i++){
        buffer[i] = Measurement(mb.buffer[i]);
    }
}

// ***  M E T H O D S  *** //
// *********************** //
void MeasurementsBuffer::add(Measurement m) {

	// ############# BEGIN Add measurement to measurements buffer ############
	buffer[nextInsertIndex] = m;

	nextInsertIndex++;

	if (nextInsertIndex >= this->getSize() - 1) {
		nextInsertIndex = 1;
	}
	// ############# END Add measurement to measurements buffer ############
}

Measurement MeasurementsBuffer::getEntryAt(int index) {
	return buffer[index];
}

int MeasurementsBuffer::getLastRecordedPointIndex() {
	return nextInsertIndex - 1;
}

int MeasurementsBuffer::getSize() {
	return sizeof(buffer) / sizeof(*buffer);
}