#pragma once

#include <string>
#include <Voxel.h>

#include <boost/serialization/map.hpp>

/**
 * @author Alberto M. Esmoris Pena
 * @verison 1.0
 *
 * @brief Class which extends Voxel to support AMAPVox format with extra
 * features
 */
class DetailedVoxel : public Voxel{
    // ***  BOOST SERIALIZATION  *** //
    // ***************************** //
    friend class boost::serialization::access;
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int version){
        boost::serialization::void_cast_register<DetailedVoxel, Voxel>();
        ar & boost::serialization::base_object<Voxel>(*this);
        ar & intValues;
        ar & doubleValues;
        ar & identifiers;
        ar & maxPad;
    }
protected:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    /**
     * @brief All integers defining the detailed voxel
     *
     * By default first integer must be total echoes count inside the voxel
     * (nbEchos), while second integer must be number of pulses entering
     * the voxel (nbSampling)
     */
    std::vector<int> intValues;
    /**
     * @brief All decimals defining the detailed voxel
     *
     * By default, the first elements must be:
     * <ol start="0">
     * <li>Plant area density</li>
     * <li>Mean inclination angle of shots which entered the voxel</li>
     * <li>Sum of weighted fractions of entering laser pulses</li>
     * <li>Potential</li>
     * <li>Distance from voxel center to the ground. If no ground is set,
     * then the plane with equation Z=0 will be used.</li>
     * <li>Mean length of optical path inside the voxel</li>
     * <li>Length sum of optical path inside the voxel</li>
     * <li>Transmittance (standarized per one meter optical path length)</li>
     * <li>Attenuation</li>
     * <li>AttenuationBiasCorrection</li>
     * </ol>
     */
    std::vector<double> doubleValues;

    /**
     * @brief Identifiers for doubleValues vector.
     */
    std::map<std::string, size_t> identifiers = std::map<std::string, size_t>({
        {"PadBVTotal", 0}, {"angleMean", 1}, {"bsEntering", 2},
        {"bsIntercepted", 3}, {"bsPotential", 4}, {"ground_distance", 5},
        {"lMeanTotal", 6}, {"lgTotal", 7}, {"transmittance", 8},
        {"attenuation", 9}, {"attenuationBiasCorrection", 10}
    });

    /**
     * @brief Maximum plant area density
     */
    double maxPad = 0.0;

public:
    // ***  CONSTRUCTION  *** //
    // ********************** //
    /**
     * @brief Defualt constructor for detailed voxel
     */
    DetailedVoxel() : Voxel() {}
    /**
     * @brief Detailed voxel constructor
     * @param center Center for the voxel
     * @param voxelSize Size for the voxel
     * @param intValues Integer values for detailed voxel
     * @param doubleValues Double values for detailed voxel
     * @see Voxel::Voxel(glm::dvec3, double)
     */
    DetailedVoxel(
        glm::dvec3 center,
        double voxelSize,
        std::vector<int> intValues,
        std::vector<double> doubleValues
    ) :
        Voxel(center, voxelSize),
        intValues(std::move(intValues)),
        doubleValues(std::move(doubleValues))
    {}
    /**
     * @brief Detailed voxel constructor
     * @param x X coordinate of voxel center
     * @param y Y coordinate of voxel center
     * @param z Z coordinate of voxel center
     * @param halfVoxelSize Half the voxel size
     * @param intValues Integer values for detailed voxel
     * @param doubleValues Double values for detailed voxel
     * @see Voxel::Voxel(double, double, double, double)
     */
    DetailedVoxel(
        double x,
        double y,
        double z,
        double halfVoxelSize,
        std::vector<int> intValues,
        std::vector<double> doubleValues
    ) :
        Voxel(x, y, z, halfVoxelSize),
        intValues(std::move(intValues)),
        doubleValues(std::move(doubleValues))
    {}

    /**
     * @see Primitive::clone
     */
    Primitive* clone() override;
    /**
     * @see Primitive::_clone
     */
    void _clone(Primitive *p) override;



    // ***  ACCESS OPERATORS  *** //
    // ************************** //
    /**
     * @brief Obtain the value at specified index (position)
     * @param index Index of the requested value
     * @return Value at given index
     */
    inline double & operator[] (size_t index)
        {return doubleValues[index];}
    /**
     * @brief Obtain the value associated with given identifier
     * @param id Identifier associated with requested value
     * @return Value associated with given identifier
     */
    inline double &operator[] (std::string const &id)
        {return doubleValues[identifiers[id]];}

    // ***  GETTERS and SETTERS  *** //
    // ***************************** //
    /**
     * @brief Obtain the number of total echoes count inside the voxel
     * @return Number of total echoes count inside the voxel
     */
    inline int getNbEchos()
        {return intValues[0];}
    /**
     * @brief Set the number of total echoes count inside the voxel
     * @param nbEchos New number of total echoes count inside the voxel
     * @return The DetailedVoxel reference in a fluent programming fashion
     */
    inline DetailedVoxel & setNbEchos(int const nbEchos)
        {intValues[0] = nbEchos; return *this;}
    /**
     * @brief Obtain the number of pulses entering the voxel
     * @return Number of pulses entering the voxel
     */
    inline int getNbSampling()
        {return intValues[1];}
    /**
     * @brief Set the number of pulses entering the voxel
     * @param nbSampling Number of pulses entering the voxel
     * @return The DetailedVoxel reference in a fluent programming fashion
     */
    inline DetailedVoxel & setNbSampling(int nbSampling)
        {intValues[1] = nbSampling; return *this;}

    // *** GETTERS and SETTERS *** //
    // *************************** //
    /**
     * @brief Obtain the number of integer values defining the detailed voxel
     * @return Number of integer values defining the detailed voxel
     */
    inline size_t getNumberOfIntValues() const
        {return intValues.size();}
    /**
     * @brief Obtain the number of double values defining the detailed voxel
     * @return Number of double value defining the detailed voxel
     */
    inline size_t getNumberOfDoubleValues() const
        {return doubleValues.size();}
    /**
     * @brief Set integer value at given index
     * @param index Index of integer value to set
     * @param value New integer value
     * @return Reference to the detailed voxel (fluent programming)
     */
    inline DetailedVoxel & setIntValue(size_t index, int value)
        {intValues[index] = value; return *this;}
    /**
     * @brief Obtain integer value at given index
     * @param index Index of integer value to obtain
     * @return Integer value at given index
     */
    inline int getIntValue(size_t index) const
        {return intValues[index];}
    /**
     * @brief Set the double value at given index
     * @param index Index of double value to set
     * @param value New double value
     * @return Reference to the
     */
    inline DetailedVoxel & setDoubleValue(size_t index, double value)
        {doubleValues[index] = value; return *this;}
    /**
     * @brief Get the double value at given index
     * @param index Index of double value to obtain
     * @return Double value at given index
     */
    inline double getDoubleValue(size_t index) const
        {return doubleValues[index];}
    /**
     * @brief The the max pad value
     * @return Max pad value
     */
    inline double getMaxPad() const {return maxPad;}
    /**
     * @brief Set the max pad value
     * @param maxPad New max pad value
     */
    inline void setMaxPad(double maxPad) {this->maxPad = maxPad;}

    // ***  M E T H O D S  *** //
    // *********************** //
    /**
     * @brief Specify DetailedVoxel can handle intersections
     * @return True
     * @see Primitive::canHandleIntersections
     */
    inline bool canHandleIntersections() override {return true;}
    /**
     * @brief Define ray intersection handling for DetailedVoxel primitive
     * @see Primitive::onRayIntersection
     */
    IntersectionHandlingResult onRayIntersection(
        NoiseSource<double> &uniformNoiseSource,
        glm::dvec3 &rayDirection,
        glm::dvec3 const &insideIntersectionPoint,
        glm::dvec3 const &outsideIntersectionPoint,
        double rayIntensity
    ) override;

    /**
     * @brief Transmittive handler for ray intersections
     */
    IntersectionHandlingResult onRayIntersectionTransmittive(
        NoiseSource<double> &uniformNoiseSource,
        glm::dvec3 &rayDirection,
        glm::dvec3 const &insideIntersectionPoint,
        glm::dvec3 const &outsideIntersectionPoint,
        double rayIntensity
    );
    /**
     * @brief Scaled handler for ray intersections
     */
    IntersectionHandlingResult onRayIntersectionScaled(
        NoiseSource<double> &uniformNoiseSource,
        glm::dvec3 &rayDirection,
        glm::dvec3 const &insideIntersectionPoint,
        glm::dvec3 const &outsideIntersectionPoint,
        double rayIntensity,
        double scaleFactor
    );
    /**
     * @brief Fixed handler for ray intersections
     */
    IntersectionHandlingResult onRayIntersectionFixed(
        NoiseSource<double> &uniformNoiseSource,
        glm::dvec3 &rayDirection,
        glm::dvec3 const &insideIntersectionPoint,
        glm::dvec3 const &outsideIntersectionPoint,
        double rayIntensity,
        double fixedSize
    );

    /**
     * @brief Configure DetailedVoxel size for scaled mode
     */
    void onFinishLoading(NoiseSource<double> &uniformNoiseSource) override;

    // ***  S I G M A  *** //
    // ******************* //
    /**
     * @see Primitive::canComputeSigmaWithLadLut
     */
    bool canComputeSigmaWithLadLut() override
        {return part->ladlut != nullptr;}
    /**
     * @see Primitive::computeSigmaWithLadLut
     */
    double computeSigmaWithLadLut(glm::dvec3 const &direction) override;
};

// ***  BOOST SERIALIZATION  *** //
// ***************************** //
/*
 * Below code is commented because it is not necessary and it might cause
 * memory leaks when called from external sources (i.e. from python)
 */
/*namespace boost{
namespace serialization{
    template<class Archive>
    inline void save_construct_data(
        Archive &ar, const DetailedVoxel *t, const unsigned int file_version
    ){
        ar << t->v.pos.x << t->v.pos.y << t->v.pos.z;
        ar << t->halfSize;
        ar << t->getMaxPad();
        ar << t->getNumberOfIntValues();
        for(size_t i = 0 ; i < t->getNumberOfIntValues() ; i++)
            ar << t->getIntValue(i);
        ar << t->getNumberOfDoubleValues();
        for(size_t i = 0 ; i < t->getNumberOfDoubleValues() ; i++)
            ar << t->getDoubleValue(i);
        ar << t->material;
    }

    template<class Archive>
    inline void load_construct_data(
        Archive &ar, DetailedVoxel *t, const unsigned int file_version
    ){
        double x = 0.0, y = 0.0, z = 0.0;
        double halfVoxelSize = 0.0;
        double maxPad = 0.0;
        size_t nIntValues = 0;
        int intVal = 0;
        std::vector<int> intValues;
        size_t nDoubleValues = 0;
        double doubleVal = 0.0;
        std::vector<double> doubleValues;
        std::shared_ptr<Material> mat;

        ar >> x; ar >> y; ar >> z;
        ar >> halfVoxelSize;
        ar >> maxPad;
        ar >> nIntValues;
        for(size_t i = 0 ; i < nIntValues ; i++){
            ar >> intVal;
            intValues.push_back(intVal);
        }
        ar >> nDoubleValues;
        for(size_t i = 0 ; i < nDoubleValues ; i++){
            ar >> doubleVal;
            doubleValues.push_back(doubleVal);
        }
        ar >> mat;

        ::new(t)DetailedVoxel(x, y, z, halfVoxelSize, intValues, doubleValues);
        t->setMaxPad(maxPad);
        t->material = mat;
    }
}
}*/