#include "Material.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Material::Material(const Material &mat){
    this->name = mat.name;
    this->isGround = mat.isGround;
    this->useVertexColors = mat.useVertexColors;;
    this->matFilePath = mat.matFilePath;
    this->map_Kd = mat.map_Kd;
    this->reflectance = mat.reflectance;
    this->specularity = mat.specularity;
    this->specularExponent = mat.specularExponent;
    this->classification = mat.classification;
    this->spectra = mat.spectra;

    for(size_t i = 0 ; i < 4 ; i++){
        this->ka[i] = mat.ka[i];
        this->kd[i] = mat.kd[i];
        this->ks[i] = mat.ks[i];
    }
}


// ***  M E T H O D S  *** //
// *********************** //
float* Material::getKd(float factor){
	float* result = new float[3];
	result[0] = kd[0] * factor;
	result[1] = kd[1] * factor;
	result[2] = kd[2] * factor;
	return result;
}

void Material::setSpecularity() {
	double kdSum = kd[0] + kd[1] + kd[2];
	double ksSum = ks[0] + ks[1] + ks[2];
	double dsSum = kdSum + ksSum;
	if (dsSum > 0) {
		specularity = ksSum / dsSum;
	}
}
