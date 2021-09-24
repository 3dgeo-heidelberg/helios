#include "AbstractGeometryFilter.h"
#include "MaterialsFileReader.h"
#include "UniformNoiseSource.h"

std::shared_ptr<Material> AbstractGeometryFilter::getMaterial(
    std::string materialName
){
	std::map<std::string, Material>::iterator it = materials.find(materialName);

	if (it == materials.end()) {
		std::shared_ptr<Material> mat = std::make_shared<Material>();
		mat->name = materialName;
		materials.insert(std::pair<std::string, Material>(materialName, *mat));
		return mat;
	}

	return std::make_shared<Material>(it->second);
}

std::vector<std::shared_ptr<Material>> AbstractGeometryFilter::parseMaterials()
{
    if(params.find("matfile") == params.end()){
        return std::vector<std::shared_ptr<Material>>(0);
    }

    // Pick material
    std::string matfile = boost::get<std::string>(params["matfile"]);
    std::map<std::string, Material> mats =
        MaterialsFileReader::loadMaterials(matfile);
    std::vector<std::shared_ptr<Material>> matvec(0);
    if(params.find("matname") != params.end()){ // Pick by name
        std::string matname = boost::get<std::string>(params["matname"]);
        std::map<std::string, Material>::iterator it = mats.find(matname);
        matvec.push_back(std::make_shared<Material>(it->second));
    }
    else { // No name, so pick first
        matvec.push_back(std::make_shared<Material>(mats.begin()->second));
    }

    // Generate randomized materials if requested
    if(params.find("randomMaterials") != params.end()){
        int nRandom = boost::get<int>(params["randomMaterials"]);
        double randomRange = 1.0;
        if(params.find("randomRange") != params.end()){
            randomRange = boost::get<double>(params["randomRange"]);
        }
        UniformNoiseSource<double> uns(*DEFAULT_RG, -randomRange, randomRange);
        std::shared_ptr<Material> mat0 = matvec[0];
        std::shared_ptr<Material> matN;
        for(int i = 0 ; i < nRandom ; i++){
            matN = std::make_shared<Material>(*mat0);
            for(size_t j = 0 ; j < 4 ; j++) {
                matN->ka[j] += uns.next();
                if(matN->ka[j] > 1.0) matN->ka[j] = 1.0;
                if(matN->ka[j] < 0.0) matN->ka[j] = 0.0;
                matN->kd[j] += uns.next();
                if(matN->kd[j] > 1.0) matN->kd[j] = 1.0;
                if(matN->kd[j] < 0.0) matN->kd[j] = 0.0;
                matN->ks[j] += uns.next();
                if(matN->ks[j] > 1.0) matN->ks[j] = 1.0;
                if(matN->ks[j] < 0.0) matN->ks[j] = 0.0;
            }
            matN->setSpecularity();
            matvec.push_back(matN);
        }
    }

    return matvec;
}
