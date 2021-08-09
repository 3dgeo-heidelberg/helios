#pragma once

#include <noise/NormalNoiseSource.h>
#include <noise/UniformNoiseSource.h>

namespace HeliosTests{


/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Test for noise generation
 */
class NoiseTest : public BaseTest {
public:
    // ***  CONSTRUCTION  *** //
    // ********************** //
    /**
     * @brief Noise test constructor
     */
    NoiseTest() : BaseTest("Noise sources test"){}

    // ***  R U N  *** //
    // *************** //
    /**
     * @see BaseTest::run
     */
    bool run() override;
};

bool NoiseTest::run(){
    // Randomness generators
    RandomnessGenerator<double> rg1(2.0);
    RandomnessGenerator<double> rg2(3.0);

    // Noise sources
    UniformNoiseSource<double> uns1(rg1);
    UniformNoiseSource<double> uns2(rg2);
    UniformNoiseSource<double> uns3(rg1);
    UniformNoiseSource<double> uns4(rg1);
    UniformNoiseSource<double> uns5(rg2);
    NormalNoiseSource<double> nns1(rg1);
    NormalNoiseSource<double> nns2(rg2);
    NormalNoiseSource<double> nns3(rg1);
    NormalNoiseSource<double> nns4(rg1);


    // Configuration
    uns3.setFixedLifespan(3); // Fixed, renew each 3 uses
    uns4.setFixedLifespan(0); // Fixed, eternal
    uns4.fixedRenew();
    uns5.configureUniformNoise(-3, 3);

    // Vars
    double cache;


    // Noise tests
    if(uns1.next()!=uns3.next()) return false;
    if(uns2.next()==uns3.next()) return false;
    if(uns3.next()!=uns4.next()) return false;
    if(uns1.next()!=uns3.next()) return false;
    if(uns1.next()==uns3.next()) return false;
    if(nns4.next()==uns4.next()) return false;
    if(nns4.next()==nns4.next()) return false;
    if(uns2.next()==uns2.next()) return false;

    cache = uns4.next();
    if(uns4.next()!=cache) return false;
    uns4.fixedRenew();
    if(uns4.next()==cache) return false;
    if(uns4.next()!=uns4.next()) return false;

    nns1.configureNormalNoise(4.0, 2.0);
    nns3.configureNormalNoise(4.0, 2.0);
    if(nns1.next()!=nns3.next()) return false;
    nns1.next();
    if(nns1.next()==nns3.next()) return false;

    for(size_t i = 0 ; i < 32 ; i++){
        cache = uns5.next();
        if(cache < -3.0 || cache > 3.0) return false;
    }
    uns5.setClipEnabled(true).setClipMin(-1.0).setClipMax(1.0); // Enable clip
    for(size_t i = 0 ; i < 32 ; i++){
        cache = uns5.next();
        if(cache < -1.0 || cache > 1.0) return false;
    }

    // Copy-move tests
    UniformNoiseSource<double> uns1c = uns1;
    if(uns1.next() != uns1c.next()) return false;
    uns1c.next();
    if(uns1.next() == uns1c.next()) return false;
    uns1.next();
    UniformNoiseSource<double> uns1m = std::move(uns1c);
    if(uns1.next() != uns1m.next()) return false;
    uns1m.next();
    if(uns1.next() == uns1m.next()) return false;
    NormalNoiseSource<double> nns1c(nns1);
    if(nns1.next() != nns1c.next()) return false;
    nns1c.next();
    if(nns1.next() == nns1c.next()) return false;
    nns1.next();
    NormalNoiseSource<double> nns1m(std::move(nns1c));
    if(nns1.next() != nns1m.next()) return false;
    nns1m.next();
    if(nns1.next() == nns1m.next()) return false;


    // Successfully reached end of test
    return true;
}

}