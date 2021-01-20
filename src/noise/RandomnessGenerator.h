#pragma once

#include <random>
#include <memory>
#include <HeliosException.h>

/**
 * @brief Set the default randomness generator
 *
 * @param seed Seed for the default randomness generator. If it is an empty
 * string, then auto seed mode will be used.
 */
void setDefaultRandomnessGeneratorSeed(std::string const seed = "");

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 *
 * @brief Class to generate random numbers
 *
 * @tparam RealType Type of the generated real random numbers.
 * For instance double or float.
 */
template <typename RealType>
class RandomnessGenerator{
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief RandomnessGenerator mode
     *
     * Specify the mode for the RandomnessGenerator.<br/>
     * Available modes are: <br/>
     * <b>AUTO_SEED</b> : Use and automatically computed seed<br/>
     * <b>FIXED_SEED_DOUBLE</b> : Use a fixed double decimal seed.<br/>
     * <b>FIXED_SEED_LONG</b> : Use a fixed long integer seed.<br/
     */
    std::string mode;

    /**
     * @brief Double seed for randomness
     */
    double doubleSeed = 0;
    /**
     * @brief Long seed for randomness
     */
    long longSeed = 0;

    /**
     * @brief Uniform Real Distribution Generator
     */
    std::unique_ptr<std::mt19937> urdGen = nullptr;
    /**
     * @brief Uniform Real Distribution
     */
    std::unique_ptr<std::uniform_real_distribution<RealType>> urd = nullptr;

    /**
     * @brief Normal Distribution Generator
     */
    std::unique_ptr<std::mt19937> ndGen = nullptr;
    /**
     * @brief Normal Distribution
     */
    std::unique_ptr<std::normal_distribution<RealType>> nd = nullptr;


    // ***  INTERNAL SEED FUNCTIONS  *** //
    // ********************************* //
    /**
     * @brief Obtain the seed in double format.
     * This getter is expected to be used with mode FIXED_SEED_DOUBLE
     *
     * @return Seed in double format
     */
    double getDoubleSeed(){return doubleSeed;}
    /**
     * @brief Obtain the seed in long format.
     * This getter is expected to be used with mode FIXED_SEED_LONG
     *
     * @return Seed in long format
     */
    long getLongSeed(){return longSeed;}
public:
    // ***  CONSTRUCTION  *** //
    // ********************** //
    /**
     * @brief Creates a RandomnessGenerator which will use an automatically
     * computed seed.
     */
    RandomnessGenerator():mode("AUTO_SEED"){};

    /**
     * @brief Creates a RandomnessGenerator which will use a double seed
     * @param seed The double seed to be used
     */
    explicit RandomnessGenerator(double seed) :
        mode("FIXED_SEED_DOUBLE"),
        doubleSeed(seed)
    {}
    /**
     * @brief Like RandomnessGenerator(double) constructor, the float is
     * casted to a double.
     * @see RandomnessGenerator::RandomnessGenerator(double)
     */
    explicit RandomnessGenerator(float seed) :
        mode("FIXED_SEED_DOUBLE"),
        doubleSeed((double)seed)
    {}

    /**
     * @brief Creates a RandomnessGenerator which will use a long seed
     * @param seed The long seed to be used
     */
    explicit RandomnessGenerator(long seed) :
        mode("FIXED_SEED_LONG"),
        longSeed(seed)
    {}
    /**
     * @brief Like RandomnessGenerator(long) constructor, the integer is
     * casted to a long.
     * @see RandomnessGenerator::RandomnessGenerator(double)
     */
    explicit RandomnessGenerator(int seed) :
        mode("FIXED_SEED_LONG"),
        longSeed((long)seed)
    {}

    /**
     * @brief Creates a RandomnessGenerator by parsing the string to
     * automatically extract the seed and define the mode
     * @param seedstr String containing the seed. It can be a long integer,
     * a double decimal or a timestamp string in "YYYY-mm-DD HH:MM:SS" format.
     * <br/>
     * NOTICE timestamp CANNOT specify a date before 1970-01-01 00:00:00.
     * It would lead to undefined behaviors.
     */
    explicit RandomnessGenerator(std::string const &seedstr);

    /**
     * @brief Copy constructor for RandomnessGenerator
     *
     * Internal components will be copied preserving their status. So,
     * when obtaining the next value from a distribution, it will be computed
     * starting at the last considered value for the copied randomness
     * generator.
     *
     * @param rg RandomnessGenerator to be copied
     */
    RandomnessGenerator(RandomnessGenerator const &rg);

    /**
     * @brief Move constructor for RandomnessGenerator
     * @param rg RandomnessGenerator to be moved
     */
    RandomnessGenerator(RandomnessGenerator &&rg) noexcept;

    // ***  ASSIGNMENT OPERATORS  *** //
    // ****************************** //
    /**
     * @brief Copy assignment operator
     * @param rg RandomnessGenerator to be copied
     * @return Reference to copied RandomnessGenerator
     */
    RandomnessGenerator& operator=(RandomnessGenerator const &rg);
    /**
     * @brief Move assignment operator
     * @param rg RandomnessGenerator to be moved
     * @return Reference to moved RandomnessGenerator
     */
    RandomnessGenerator& operator=(RandomnessGenerator &&rg);

    // ***   S W A P   *** //
    // ******************* //
    /**
     * @brief Swap two RandomnessGenerator
     * @param rgA RandomnessGenerator to be swapped with rgB
     * @param rgB RandomnessGenerator to be swapped with rgA
     */
    void swap(
        RandomnessGenerator &rgA,
        RandomnessGenerator &rgB
    );

    // ***  RANDOMNESS METHODS  *** //
    // **************************** //
    /**
     * @brief Compute a uniform real distribution using the specified real
     *  data type
     * @param lowerBound No less than lower bound values are allowed
     * @param upperBound No greater than upper bound values are allowed
     * @return Uniform real distribution satisfying specified bounds
     */
    void computeUniformRealDistribution(
        RealType lowerBound, RealType upperBound
    );
    /**
     * @brief Obtain the next value in the computed uniform real distribution
     *
     * If no uniform real distribution has been computed, then it is computed
     * before obtaining the first element in the distribution.
     * When uniform real distribution is computed by this method,
     * it is bounded inside interval [0, 1]
     *
     * @return Next real number in the uniform real distribution
     */
    RealType uniformRealDistributionNext();
    /**
     * @brief Compute a normal distribution using the specified real data type
     * @param mean The mean value for the normal distribution
     * @param stdev The standard deviation value for the normal distribution
     */
    void computeNormalDistribution(RealType mean, RealType stdev);
    /**
     * @brief Obtain the next value in the computed normal distribution
     *
     * If no normal distribution has been computed, then it is computed
     * before obtaining the first element in the distribution.
     * When normal distribution is computed by this method,
     * its mean is 0 and its standard deviation is 1
     *
     * @return Next real number in the normal distribution
     */
    RealType normalDistributionNext();


};

// ***  DEFAULT RANDOMNESS GENERATOR  *** //
// ************************************** //
extern bool DEFAULT_RG_MODIFIED_FLAG;
extern std::unique_ptr<RandomnessGenerator<double>> DEFAULT_RG;




// ***   CLASS  IMPLEMENTATION   *** //
// ********************************* //

// ***  CONSTRUCTION  *** //
// ********************** //
template<typename RealType>
RandomnessGenerator<RealType>::RandomnessGenerator(std::string const &seedstr){
    bool isTimestamp = false;
    bool isDouble = false;
    for(size_t i = 0 ; i < seedstr.length() ; i++){
        if(seedstr[i] == ':') isTimestamp = true;
        if(seedstr[i] == '.') isDouble = true;
    }

    if(isTimestamp){
        mode = "FIXED_SEED_LONG";
        longSeed = 0;
        longSeed += (std::stol(seedstr.substr(0,4))-1970)*31104000;
        longSeed += std::stol(seedstr.substr(5,2))*2592000;
        longSeed += std::stol(seedstr.substr(8,2))*86400;
        longSeed += std::stol(seedstr.substr(11,2))*3600;
        longSeed += std::stol(seedstr.substr(14,2))*60;
        longSeed += std::stol(seedstr.substr(17,2));
    }
    else if(isDouble){
        mode = "FIXED_SEED_DOUBLE";
        doubleSeed = std::stod(seedstr);
    }
    else{
        mode = "FIXED_SEED_LONG";
        longSeed = std::stol(seedstr);
    }
}

template<typename RealType>
RandomnessGenerator<RealType>::RandomnessGenerator(
    RandomnessGenerator  const &rg
){
    this->mode = rg.mode;
    this->doubleSeed = rg.doubleSeed;
    this->longSeed = rg.longSeed;

    if(rg.urdGen == nullptr) this->urdGen = nullptr;
    else this->urdGen = std::unique_ptr<std::mt19937>(
            new std::mt19937(*rg.urdGen)
        );
    if(rg.urd == nullptr) this->urdGen = nullptr;
    else this->urd =
        std::unique_ptr<std::uniform_real_distribution<RealType>>(
            new std::uniform_real_distribution<RealType>(*rg.urd)
        );

    if(rg.ndGen == nullptr) this->ndGen = nullptr;
    else this->ndGen = std::unique_ptr<std::mt19937>(
            new std::mt19937(*rg.ndGen)
        );
    if(rg.nd == nullptr) this->nd = nullptr;
    else this->nd = std::unique_ptr<std::normal_distribution<RealType>>(
            new std::normal_distribution<RealType>(*rg.nd)
        );
}

template<typename RealType>
RandomnessGenerator<RealType>::RandomnessGenerator(
    RandomnessGenerator &&rg
)noexcept{
    swap(*this, rg);
}

// ***  ASSIGNMENT OPERATORS  *** //
// ****************************** //
template<typename RealType>
RandomnessGenerator<RealType>&
RandomnessGenerator<RealType>::operator=(
    RandomnessGenerator<RealType> const &rg
){
    RandomnessGenerator<RealType> rgCopy(rg);
    swap(*this, rgCopy);
    return *this;
}
template<typename RealType>
RandomnessGenerator<RealType>&
RandomnessGenerator<RealType>::operator=(RandomnessGenerator &&rg){
    swap(*this, rg);
    return *this;
}

// ***   S W A P   *** //
// ******************* //
template<typename RealType>
void RandomnessGenerator<RealType>::swap(
    RandomnessGenerator<RealType> &rgA,
    RandomnessGenerator<RealType> &rgB
){
    std::swap(rgA.mode, rgB.mode);
    std::swap(rgA.doubleSeed, rgB.doubleSeed);
    std::swap(rgA.longSeed, rgB.longSeed);
    std::swap(rgA.urdGen, rgB.urdGen);
    std::swap(rgA.urd, rgB.urd);
    std::swap(rgA.ndGen, rgB.ndGen);
    std::swap(rgA.nd, rgB.nd);
}

// ***  RANDOMNESS METHODS  *** //
// **************************** //
template <typename RealType>
void RandomnessGenerator<RealType>::computeUniformRealDistribution(
    RealType lowerBound, RealType upperBound
){
    if(mode == "AUTO_SEED") {
        std::random_device rd;
        urdGen = std::unique_ptr<std::mt19937>(new std::mt19937(rd()));
        urd = std::unique_ptr<std::uniform_real_distribution<RealType>>(
            new std::uniform_real_distribution<RealType>(
                lowerBound, upperBound
            ));
    }
    else if(mode == "FIXED_SEED_DOUBLE"){
        urdGen = std::unique_ptr<std::mt19937>(
            new std::mt19937(getDoubleSeed())
        );
        urd = std::unique_ptr<std::uniform_real_distribution<RealType>>(
            new std::uniform_real_distribution<RealType>(
                lowerBound, upperBound
            ));
    }
    else if(mode == "FIXED_SEED_LONG"){
        urdGen = std::unique_ptr<std::mt19937>(
            new std::mt19937(getLongSeed())
        );
        urd = std::unique_ptr<std::uniform_real_distribution<RealType>>(
            new std::uniform_real_distribution<RealType>(
                lowerBound, upperBound
            ));
    }
}
template <typename RealType>
RealType RandomnessGenerator<RealType>::uniformRealDistributionNext(){
    if(urd == nullptr) computeUniformRealDistribution(0.0, 1.0);
    return (*urd)(*urdGen);
}

template <typename RealType>
void RandomnessGenerator<RealType>::computeNormalDistribution(
    RealType mean, RealType stdev
){
    if(mode == "AUTO_SEED") {
        std::random_device rd;
        ndGen = std::unique_ptr<std::mt19937>(new std::mt19937(rd()));
        nd = std::unique_ptr<std::normal_distribution<RealType>>(
            new std::normal_distribution<RealType>(
                mean, stdev
            ));
    }
    else if(mode == "FIXED_SEED_DOUBLE"){
        ndGen = std::unique_ptr<std::mt19937>(
            new std::mt19937(getDoubleSeed())
        );
        nd = std::unique_ptr<std::normal_distribution<RealType>>(
            new std::normal_distribution<RealType>(
                mean, stdev
            ));
    }
    else if(mode == "FIXED_SEED_LONG"){
        ndGen = std::unique_ptr<std::mt19937>(
            new std::mt19937(getLongSeed())
        );
        nd = std::unique_ptr<std::normal_distribution<RealType>>(
            new std::normal_distribution<RealType>(
                mean, stdev
            ));
    }
}

template <typename RealType>
RealType RandomnessGenerator<RealType>::normalDistributionNext(){
    if(nd == nullptr) computeNormalDistribution(0.0, 1.0);
    return (*nd)(*ndGen);
}
