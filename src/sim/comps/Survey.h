#pragma once

#include <Asset.h>
#include <Scanner.h>
#include <AbstractDetector.h>
#include <Leg.h>

class SurveyPlayback;

/**
 * @brief Class representing a Helios++ survey
 */
class Survey : public Asset {
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief Survey name
     */
	std::string name = "Unnamed Survey Playback";
	/**
	 * @brief Number of runs for the survey
	 */
	int numRuns = -1;
	/**
	 * @brief Scanner used by the survey
	 * @see Scanner
	 */
	std::shared_ptr<Scanner> scanner = nullptr;
	/**
	 * @brief Simulation speed factor for the survey
	 */
	double simSpeedFactor = 1;
	/**
	 * @brief All legs belonging to the survey
	 * @see Leg
	 */
	std::vector<std::shared_ptr<Leg>> legs;

private:
    /**
     * @brief Distance passing through all legs
     */
	double length = 0;	// Distance passing through all the legs

public:
    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    /**
     * @brief Survey default constructor
     */
    Survey() = default;
    Survey(Survey &survey, bool const deepCopy=false);
    ~Survey() override {
        if(scanner != nullptr) scanner->setAllDetectors(nullptr);
    }

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Add a leg to the survey at given index
     * @param insertIndex Index where the leg must be inserted
     * @param leg Leg to be added to the survey
     * @see Leg
     */
	void addLeg(int insertIndex, std::shared_ptr<Leg> leg);
	/**
	 * @brief Remove a leg from the survey
	 * @param legIndex Index of the leg to be removed from the survey
	 * @see Leg
	 */
	void removeLeg(int legIndex);

	/**
	 * @brief Compute survey length (distance passing through all legs)
	 * @see Survey::length
	 */
	void calculateLength();
	/**
	 * @brief Obtain survey length (distance passing through all legs)
	 * @return Survey length
	 * @see Survey::length
	 */
	double getLength();
	/**
	 * @brief Hatch all pending eggs
	 * @see EggAsset
	 * @see InterpolatedMovingPlatformEgg
	 */
	void hatch(SurveyPlayback &sp);
};