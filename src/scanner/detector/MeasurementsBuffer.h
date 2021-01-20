#pragma once

#include "Measurement.h"

/**
 * @brief Class abstracting a buffer of measurements
 * @see Measurement
 */
class MeasurementsBuffer {
private:
    // ***  CLASS/STATIC ATTRIBUTES  *** //
    // ********************************* //
    /**
     * @brief Size (in number of elements) of measurements buffer
     */
	static const int MEASUREMENTS_BUFFER_SIZE = 100000;

public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Index of next insertion
     */
    int nextInsertIndex = 1;
    /**
     * @brief Memory for the buffer
     */
	Measurement buffer[MEASUREMENTS_BUFFER_SIZE];

	// ***  CONSTRUCTION / DESTRUCTION  *** //
	// ************************************ //
	MeasurementsBuffer() = default;
	MeasurementsBuffer(const MeasurementsBuffer &mb);

	// ***  M E T H O D S  *** //
	// *********************** //
	/**
	 * @brief Add a measurement to the buffer
	 * @param m Measurement to be added to the buffer
	 */
	void add(Measurement m);
	/**
	 * @brief Get the measurement at given index
	 * @param index Index of measurement to obtain
	 * @return Measurement at given index
	 */
	Measurement getEntryAt(int index);
	/**
	 * @brief Get index of last insertion
	 * @return Index of last insertion
	 */
	int getLastRecordedPointIndex();
	/**
	 * @brief Obtain the size of the buffer
	 * @return Size of the buffer
	 */
	int getSize();
};
