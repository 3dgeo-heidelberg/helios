#include <Scene.h>

#include <DynMovingObject.h>
#include <GroveKDTreeRaycaster.h>
#include <KDGroveSubject.h>
#include <KDTreeFactoryMaker.h>
#include <KDTreeNodeRoot.h>
#include <scene/primitives/AABB.h>
#include <scene/primitives/DetailedVoxel.h>
#include <scene/primitives/Triangle.h>
#include <scene/primitives/Voxel.h>
#include <util/HeliosException.h>

#include <cereal/archives/binary.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace {

bool
isLikelyZlibStream(std::istream& in)
{
  std::istream::pos_type const start = in.tellg();
  if (start == std::istream::pos_type(-1))
    return false;

  unsigned char header[2] = { 0, 0 };
  in.read(reinterpret_cast<char*>(header), 2);
  std::streamsize const got = in.gcount();
  in.clear();
  in.seekg(start);
  if (got != 2)
    return false;

  // RFC 1950: CMF must encode DEFLATE with 32K window and the
  // two-byte header must be divisible by 31.
  if (header[0] != 0x78)
    return false;
  unsigned int const cmf_flg =
    (static_cast<unsigned int>(header[0]) << 8u) | header[1];
  return (cmf_flg % 31u) == 0u;
}

int
normalizeCompressionLevel(int const compressionLevel)
{
  if (compressionLevel < 0 || compressionLevel > 9) {
    throw HeliosException(
      "Scene serialization compressionLevel must be in [0, 9]");
  }
  return compressionLevel;
}

struct DVec2DTO
{
  double x = 0.0;
  double y = 0.0;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(x, y);
  }
};

struct DVec3DTO
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(x, y, z);
  }
};

struct Color4fDTO
{
  float x = 1.0f;
  float y = 1.0f;
  float z = 1.0f;
  float w = 1.0f;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(x, y, z, w);
  }
};

struct RotationDTO
{
  double q0 = 1.0;
  double q1 = 0.0;
  double q2 = 0.0;
  double q3 = 0.0;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(q0, q1, q2, q3);
  }
};

struct VertexDTO
{
  DVec3DTO pos;
  DVec3DTO normal;
  DVec2DTO texcoords;
  Color4fDTO color;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(pos, normal, texcoords, color);
  }
};

struct LadLutDTO
{
  std::vector<double> X;
  std::vector<double> Y;
  std::vector<double> Z;
  std::vector<double> G;
  std::vector<double> angles;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(X, Y, Z, G, angles);
  }
};

struct MaterialDTO
{
  std::string name = "default";
  bool isGround = false;
  bool useVertexColors = false;
  std::string matFilePath;
  std::string map_Kd;
  double reflectance = std::numeric_limits<double>::quiet_NaN();
  double specularity = 0.0;
  double specularExponent = 10.0;
  int classification = 0;
  std::string spectra;
  float ka[4] = { 0, 0, 0, 0 };
  float kd[4] = { 0, 0, 0, 0 };
  float ks[4] = { 0, 0, 0, 0 };

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(name,
       isGround,
       useVertexColors,
       matFilePath,
       map_Kd,
       reflectance,
       specularity,
       specularExponent,
       classification,
       spectra,
       ka[0],
       ka[1],
       ka[2],
       ka[3],
       kd[0],
       kd[1],
       kd[2],
       kd[3],
       ks[0],
       ks[1],
       ks[2],
       ks[3]);
  }
};

enum class PrimitiveKindDTO : int
{
  TRIANGLE = 0,
  VOXEL = 1,
  DETAILED_VOXEL = 2
};

struct PrimitiveDTO
{
  PrimitiveKindDTO kind = PrimitiveKindDTO::TRIANGLE;
  int partIndex = -1;
  int materialIndex = -1;

  std::array<VertexDTO, 3> triangleVertices;

  VertexDTO voxelVertex;
  int voxelNumPoints = 0;
  double voxelR = 0.0;
  double voxelG = 0.0;
  double voxelB = 0.0;
  Color4fDTO voxelColor;
  double voxelHalfSize = 0.0;

  std::vector<int> detailedIntValues;
  std::vector<double> detailedDoubleValues;
  double detailedMaxPad = 0.0;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(kind,
       partIndex,
       materialIndex,
       triangleVertices,
       voxelVertex,
       voxelNumPoints,
       voxelR,
       voxelG,
       voxelB,
       voxelColor,
       voxelHalfSize,
       detailedIntValues,
       detailedDoubleValues,
       detailedMaxPad);
  }
};

struct PartDTO
{
  int objectType = (int)ScenePart::ObjectType::STATIC_OBJECT;
  int primitiveType = (int)ScenePart::PrimitiveType::NONE;
  std::vector<std::size_t> primitiveIndices;

  std::vector<double> centroid;
  bool hasBound = false;
  DVec3DTO boundMin;
  DVec3DTO boundMax;

  std::string id;
  std::vector<std::size_t> subpartLimit;
  std::string onRayIntersectionMode;
  double onRayIntersectionArgument = 0.0;
  bool randomShift = false;

  bool hasLadLut = false;
  LadLutDTO ladLut;

  DVec3DTO origin;
  RotationDTO rotation;
  double scale = 1.0;
  int forceOnGround = 0;

  int dynStepInterval = 1;
  double dynTimeStep = std::numeric_limits<double>::quiet_NaN();
  int observerStepInterval = 1;
  double observerDynTimeStep = std::numeric_limits<double>::quiet_NaN();

  bool hasSwapOnRepeat = false;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(objectType,
       primitiveType,
       primitiveIndices,
       centroid,
       hasBound,
       boundMin,
       boundMax,
       id,
       subpartLimit,
       onRayIntersectionMode,
       onRayIntersectionArgument,
       randomShift,
       hasLadLut,
       ladLut,
       origin,
       rotation,
       scale,
       forceOnGround,
       dynStepInterval,
       dynTimeStep,
       observerStepInterval,
       observerDynTimeStep,
       hasSwapOnRepeat);
  }
};

struct KDTreeFactoryDescriptorDTO
{
  int factoryType = 1;
  std::size_t numJobs = 1;
  std::size_t geomJobs = 1;
  std::size_t lossNodes = 32;
  double interiorCost = 1.0;
  double leafCost = 1.0;
  double objectCost = 1.0;
  bool buildLightNodes = true;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(factoryType,
       numJobs,
       geomJobs,
       lossNodes,
       interiorCost,
       leafCost,
       objectCost,
       buildLightNodes);
  }
};

struct KDNodeDTO
{
  int splitAxis = 0;
  double splitPos = 0.0;
  std::vector<std::int64_t> primitiveIndices;
  std::shared_ptr<KDNodeDTO> left = nullptr;
  std::shared_ptr<KDNodeDTO> right = nullptr;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(splitAxis, splitPos, primitiveIndices, left, right);
  }
};

struct TreeDTO
{
  int subjectPartIndex = -1;
  bool hasRoot = false;
  KDNodeDTO root;
  int statsMaxPrimsInLeaf = 0;
  int statsMinPrimsInLeaf = 0;
  int statsMaxDepth = 0;
  int statsNumInterior = 0;
  int statsNumLeaves = 0;
  double statsTotalCost = 0.0;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(subjectPartIndex,
       hasRoot,
       root,
       statsMaxPrimsInLeaf,
       statsMinPrimsInLeaf,
       statsMaxDepth,
       statsNumInterior,
       statsNumLeaves,
       statsTotalCost);
  }
};

struct KDGroveDTO
{
  std::vector<TreeDTO> trees;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(trees);
  }
};

struct SceneDTO
{
  std::string assetId;
  std::string assetName;
  std::string sourceFilePath;
  double defaultReflectance = 50.0;

  bool hasBBox = false;
  DVec3DTO bboxMin;
  DVec3DTO bboxMax;

  bool hasBBoxCRS = false;
  DVec3DTO bboxCrsMin;
  DVec3DTO bboxCrsMax;

  std::vector<MaterialDTO> materials;
  std::vector<PrimitiveDTO> primitives;
  std::vector<PartDTO> parts;

  bool hasFactory = false;
  KDTreeFactoryDescriptorDTO factory;

  bool hasKDGrove = false;
  KDGroveDTO kdgrove;

  template<class Archive>
  void serialize(Archive& ar)
  {
    ar(assetId,
       assetName,
       sourceFilePath,
       defaultReflectance,
       hasBBox,
       bboxMin,
       bboxMax,
       hasBBoxCRS,
       bboxCrsMin,
       bboxCrsMax,
       materials,
       primitives,
       parts,
       hasFactory,
       factory,
       hasKDGrove,
       kdgrove);
  }
};

inline DVec2DTO
toDTO(glm::dvec2 const& v)
{
  DVec2DTO dto;
  dto.x = v.x;
  dto.y = v.y;
  return dto;
}

inline DVec3DTO
toDTO(glm::dvec3 const& v)
{
  DVec3DTO dto;
  dto.x = v.x;
  dto.y = v.y;
  dto.z = v.z;
  return dto;
}

inline glm::dvec2
fromDTO(DVec2DTO const& v)
{
  return glm::dvec2(v.x, v.y);
}

inline glm::dvec3
fromDTO(DVec3DTO const& v)
{
  return glm::dvec3(v.x, v.y, v.z);
}

inline Color4fDTO
toDTO(Color4f const& c)
{
  Color4fDTO dto;
  dto.x = c.x;
  dto.y = c.y;
  dto.z = c.z;
  dto.w = c.w;
  return dto;
}

inline Color4f
fromDTO(Color4fDTO const& c)
{
  return Color4f(c.x, c.y, c.z, c.w);
}

inline RotationDTO
toDTO(Rotation const& r)
{
  RotationDTO dto;
  dto.q0 = r.getQ0();
  dto.q1 = r.getQ1();
  dto.q2 = r.getQ2();
  dto.q3 = r.getQ3();
  return dto;
}

inline Rotation
fromDTO(RotationDTO const& r)
{
  return Rotation(r.q0, r.q1, r.q2, r.q3, false);
}

inline VertexDTO
toDTO(Vertex const& v)
{
  VertexDTO dto;
  dto.pos = toDTO(v.pos);
  dto.normal = toDTO(v.normal);
  dto.texcoords = toDTO(v.texcoords);
  dto.color = toDTO(v.color);
  return dto;
}

inline Vertex
fromDTO(VertexDTO const& v)
{
  Vertex out;
  out.pos = fromDTO(v.pos);
  out.normal = fromDTO(v.normal);
  out.texcoords = fromDTO(v.texcoords);
  out.color = fromDTO(v.color);
  return out;
}

inline MaterialDTO
toDTO(Material const& m)
{
  MaterialDTO dto;
  dto.name = m.name;
  dto.isGround = m.isGround;
  dto.useVertexColors = m.useVertexColors;
  dto.matFilePath = m.matFilePath;
  dto.map_Kd = m.map_Kd;
  dto.reflectance = m.reflectance;
  dto.specularity = m.specularity;
  dto.specularExponent = m.specularExponent;
  dto.classification = m.classification;
  dto.spectra = m.spectra;
  for (int i = 0; i < 4; ++i) {
    dto.ka[i] = m.ka[i];
    dto.kd[i] = m.kd[i];
    dto.ks[i] = m.ks[i];
  }
  return dto;
}

inline std::shared_ptr<Material>
fromDTO(MaterialDTO const& dto)
{
  std::shared_ptr<Material> m = std::make_shared<Material>();
  m->name = dto.name;
  m->isGround = dto.isGround;
  m->useVertexColors = dto.useVertexColors;
  m->matFilePath = dto.matFilePath;
  m->map_Kd = dto.map_Kd;
  m->reflectance = dto.reflectance;
  m->specularity = dto.specularity;
  m->specularExponent = dto.specularExponent;
  m->classification = dto.classification;
  m->spectra = dto.spectra;
  for (int i = 0; i < 4; ++i) {
    m->ka[i] = dto.ka[i];
    m->kd[i] = dto.kd[i];
    m->ks[i] = dto.ks[i];
  }
  return m;
}

inline LadLutDTO
toDTO(LadLut const& lad)
{
  LadLutDTO dto;
  dto.X = lad.X;
  dto.Y = lad.Y;
  dto.Z = lad.Z;
  dto.G = lad.G;
  dto.angles = lad.angles;
  return dto;
}

inline std::shared_ptr<LadLut>
fromDTO(LadLutDTO const& dto)
{
  std::shared_ptr<LadLut> lad = std::make_shared<LadLut>();
  lad->X = dto.X;
  lad->Y = dto.Y;
  lad->Z = dto.Z;
  lad->G = dto.G;
  lad->angles = dto.angles;
  return lad;
}

inline PrimitiveKindDTO
getPrimitiveKind(Primitive const* primitive)
{
  if (dynamic_cast<DetailedVoxel const*>(primitive) != nullptr)
    return PrimitiveKindDTO::DETAILED_VOXEL;
  if (dynamic_cast<Voxel const*>(primitive) != nullptr)
    return PrimitiveKindDTO::VOXEL;
  if (dynamic_cast<Triangle const*>(primitive) != nullptr)
    return PrimitiveKindDTO::TRIANGLE;

  throw HeliosException(
    "Scene cereal serialization does not support this primitive type");
}

PrimitiveDTO
primitiveToDTO(Primitive const* primitive,
               int const partIndex,
               int const materialIndex)
{
  PrimitiveDTO dto;
  dto.kind = getPrimitiveKind(primitive);
  dto.partIndex = partIndex;
  dto.materialIndex = materialIndex;

  if (dto.kind == PrimitiveKindDTO::TRIANGLE) {
    Triangle const* tri = static_cast<Triangle const*>(primitive);
    for (int i = 0; i < 3; ++i)
      dto.triangleVertices[(std::size_t)i] = toDTO(tri->verts[i]);
    return dto;
  }

  if (dto.kind == PrimitiveKindDTO::VOXEL ||
      dto.kind == PrimitiveKindDTO::DETAILED_VOXEL) {
    Voxel const* vox = static_cast<Voxel const*>(primitive);
    dto.voxelVertex = toDTO(vox->v);
    dto.voxelNumPoints = vox->numPoints;
    dto.voxelR = vox->r;
    dto.voxelG = vox->g;
    dto.voxelB = vox->b;
    dto.voxelColor = toDTO(vox->color);
    dto.voxelHalfSize = vox->halfSize;
  }

  if (dto.kind == PrimitiveKindDTO::DETAILED_VOXEL) {
    DetailedVoxel const* dv = static_cast<DetailedVoxel const*>(primitive);
    dto.detailedIntValues.reserve(dv->getNumberOfIntValues());
    for (std::size_t i = 0; i < dv->getNumberOfIntValues(); ++i)
      dto.detailedIntValues.push_back(dv->getIntValue(i));
    dto.detailedDoubleValues.reserve(dv->getNumberOfDoubleValues());
    for (std::size_t i = 0; i < dv->getNumberOfDoubleValues(); ++i)
      dto.detailedDoubleValues.push_back(dv->getDoubleValue(i));
    dto.detailedMaxPad = dv->getMaxPad();
  }

  return dto;
}

Primitive*
primitiveFromDTO(PrimitiveDTO const& dto,
                 std::vector<std::shared_ptr<Material>> const& materials)
{
  Primitive* out = nullptr;

  if (dto.kind == PrimitiveKindDTO::TRIANGLE) {
    out = new Triangle(fromDTO(dto.triangleVertices[0]),
                       fromDTO(dto.triangleVertices[1]),
                       fromDTO(dto.triangleVertices[2]));
  } else if (dto.kind == PrimitiveKindDTO::VOXEL) {
    Voxel* vox = new Voxel(dto.voxelVertex.pos.x,
                           dto.voxelVertex.pos.y,
                           dto.voxelVertex.pos.z,
                           dto.voxelHalfSize);
    vox->v = fromDTO(dto.voxelVertex);
    vox->numPoints = dto.voxelNumPoints;
    vox->r = dto.voxelR;
    vox->g = dto.voxelG;
    vox->b = dto.voxelB;
    vox->color = fromDTO(dto.voxelColor);
    vox->halfSize = dto.voxelHalfSize;
    vox->update();
    out = vox;
  } else if (dto.kind == PrimitiveKindDTO::DETAILED_VOXEL) {
    DetailedVoxel* dv = new DetailedVoxel(dto.voxelVertex.pos.x,
                                          dto.voxelVertex.pos.y,
                                          dto.voxelVertex.pos.z,
                                          dto.voxelHalfSize,
                                          dto.detailedIntValues,
                                          dto.detailedDoubleValues);
    dv->v = fromDTO(dto.voxelVertex);
    dv->numPoints = dto.voxelNumPoints;
    dv->r = dto.voxelR;
    dv->g = dto.voxelG;
    dv->b = dto.voxelB;
    dv->color = fromDTO(dto.voxelColor);
    dv->halfSize = dto.voxelHalfSize;
    dv->setMaxPad(dto.detailedMaxPad);
    dv->update();
    out = dv;
  } else {
    throw HeliosException("Unexpected primitive kind while loading scene");
  }

  if (dto.materialIndex >= 0) {
    std::size_t const materialIdx = (std::size_t)dto.materialIndex;
    if (materialIdx >= materials.size()) {
      delete out;
      throw HeliosException("Invalid material index while loading scene");
    }
    out->material = materials[materialIdx];
  }

  return out;
}

PartDTO
partToDTO(std::shared_ptr<ScenePart> const& part,
          std::unordered_map<Primitive const*, std::size_t> const& primIndexMap)
{
  PartDTO dto;
  dto.objectType = (int)part->getType();
  dto.primitiveType = (int)part->primitiveType;
  dto.id = part->mId;
  dto.subpartLimit = part->subpartLimit;
  dto.onRayIntersectionMode = part->onRayIntersectionMode;
  dto.onRayIntersectionArgument = part->onRayIntersectionArgument;
  dto.randomShift = part->randomShift;
  dto.origin = toDTO(part->mOrigin);
  dto.rotation = toDTO(part->mRotation);
  dto.scale = part->mScale;
  dto.forceOnGround = part->forceOnGround;
  dto.hasSwapOnRepeat = part->sorh != nullptr;

  if (part->centroid.n_elem > 0) {
    dto.centroid.reserve(part->centroid.n_elem);
    for (arma::uword i = 0; i < part->centroid.n_elem; ++i)
      dto.centroid.push_back(part->centroid[i]);
  }

  if (part->bound != nullptr) {
    dto.hasBound = true;
    dto.boundMin = toDTO(part->bound->getMin());
    dto.boundMax = toDTO(part->bound->getMax());
  }

  if (part->ladlut != nullptr) {
    dto.hasLadLut = true;
    dto.ladLut = toDTO(*part->ladlut);
  }

  if (DynObject const* dynObj = dynamic_cast<DynObject const*>(part.get())) {
    dto.dynStepInterval = dynObj->getStepInterval();
    dto.dynTimeStep = dynObj->getDynTimeStep();
  }
  if (DynMovingObject const* moving =
        dynamic_cast<DynMovingObject const*>(part.get())) {
    dto.observerStepInterval = moving->getObserverStepInterval();
    dto.observerDynTimeStep = moving->getObserverDynTimeStep();
  }

  dto.primitiveIndices.reserve(part->mPrimitives.size());
  for (Primitive const* primitive : part->mPrimitives) {
    auto const it = primIndexMap.find(primitive);
    if (it == primIndexMap.end()) {
      throw HeliosException(
        "Scene part primitive is not registered in scene primitive list");
    }
    dto.primitiveIndices.push_back(it->second);
  }

  return dto;
}

std::shared_ptr<ScenePart>
partFromDTO(PartDTO const& dto)
{
  if (dto.hasSwapOnRepeat) {
    throw HeliosException(
      "Scene cereal serialization does not support SwapOnRepeatHandler yet");
  }

  std::shared_ptr<ScenePart> part = nullptr;
  ScenePart::ObjectType const objectType =
    static_cast<ScenePart::ObjectType>(dto.objectType);

  if (objectType == ScenePart::ObjectType::STATIC_OBJECT) {
    part = std::make_shared<ScenePart>();
  } else if (objectType == ScenePart::ObjectType::DYN_MOVING_OBJECT) {
    part = std::make_shared<DynMovingObject>();
  } else {
    throw HeliosException(
      "Scene cereal serialization does not support abstract DynObject "
      "deserialization");
  }

  part->primitiveType =
    static_cast<ScenePart::PrimitiveType>(dto.primitiveType);
  part->mId = dto.id;
  part->subpartLimit = dto.subpartLimit;
  part->onRayIntersectionMode = dto.onRayIntersectionMode;
  part->onRayIntersectionArgument = dto.onRayIntersectionArgument;
  part->randomShift = dto.randomShift;
  part->mOrigin = fromDTO(dto.origin);
  part->mRotation = fromDTO(dto.rotation);
  part->mScale = dto.scale;
  part->forceOnGround = dto.forceOnGround;
  part->mCrs = nullptr;
  part->mEnv = nullptr;

  if (!dto.centroid.empty()) {
    part->centroid = arma::colvec(dto.centroid.size());
    for (std::size_t i = 0; i < dto.centroid.size(); ++i)
      part->centroid[i] = dto.centroid[i];
  }

  if (dto.hasBound) {
    part->bound =
      std::make_shared<AABB>(fromDTO(dto.boundMin), fromDTO(dto.boundMax));
  }

  if (dto.hasLadLut) {
    part->ladlut = fromDTO(dto.ladLut);
  }

  if (DynObject* dynObj = dynamic_cast<DynObject*>(part.get())) {
    dynObj->setStepInterval(dto.dynStepInterval);
    dynObj->setDynTimeStep(dto.dynTimeStep);
  }
  if (DynMovingObject* moving = dynamic_cast<DynMovingObject*>(part.get())) {
    moving->setObserverStepInterval(dto.observerStepInterval);
    moving->setObserverDynTimeStep(dto.observerDynTimeStep);
  }

  return part;
}

KDTreeFactoryDescriptorDTO
factoryToDTO(std::shared_ptr<KDTreeFactory> const& kdtf)
{
  KDTreeFactoryDescriptorDTO dto;
  dto.buildLightNodes = kdtf->isBuildingLightNodes();

  if (std::shared_ptr<MultiThreadSAHKDTreeFactory> mtSah =
        std::dynamic_pointer_cast<MultiThreadSAHKDTreeFactory>(kdtf)) {
    dto.numJobs = mtSah->getNumJobs();
    dto.geomJobs = mtSah->getGeomJobs();
    std::shared_ptr<SimpleKDTreeFactory> base = mtSah->getKdtf();
    if (std::shared_ptr<FastSAHKDTreeFactory> fast =
          std::dynamic_pointer_cast<FastSAHKDTreeFactory>(base)) {
      dto.factoryType = 4;
      dto.lossNodes = fast->getLossNodes();
      dto.interiorCost = fast->getInteriorCost();
      dto.leafCost = fast->getLeafCost();
      dto.objectCost = fast->getObjectCost();
      return dto;
    }
    if (std::shared_ptr<AxisSAHKDTreeFactory> axis =
          std::dynamic_pointer_cast<AxisSAHKDTreeFactory>(base)) {
      dto.factoryType = 3;
      dto.lossNodes = axis->getLossNodes();
      dto.interiorCost = axis->getInteriorCost();
      dto.leafCost = axis->getLeafCost();
      dto.objectCost = axis->getObjectCost();
      return dto;
    }
    if (std::shared_ptr<SAHKDTreeFactory> sah =
          std::dynamic_pointer_cast<SAHKDTreeFactory>(base)) {
      dto.factoryType = 2;
      dto.lossNodes = sah->getLossNodes();
      dto.interiorCost = sah->getInteriorCost();
      dto.leafCost = sah->getLeafCost();
      dto.objectCost = sah->getObjectCost();
      return dto;
    }
    dto.factoryType = 1;
    return dto;
  }

  if (std::shared_ptr<MultiThreadKDTreeFactory> mt =
        std::dynamic_pointer_cast<MultiThreadKDTreeFactory>(kdtf)) {
    dto.factoryType = 1;
    dto.numJobs = mt->getNumJobs();
    dto.geomJobs = mt->getGeomJobs();
    return dto;
  }

  dto.numJobs = 1;
  dto.geomJobs = 1;

  if (std::shared_ptr<FastSAHKDTreeFactory> fast =
        std::dynamic_pointer_cast<FastSAHKDTreeFactory>(kdtf)) {
    dto.factoryType = 4;
    dto.lossNodes = fast->getLossNodes();
    dto.interiorCost = fast->getInteriorCost();
    dto.leafCost = fast->getLeafCost();
    dto.objectCost = fast->getObjectCost();
    return dto;
  }
  if (std::shared_ptr<AxisSAHKDTreeFactory> axis =
        std::dynamic_pointer_cast<AxisSAHKDTreeFactory>(kdtf)) {
    dto.factoryType = 3;
    dto.lossNodes = axis->getLossNodes();
    dto.interiorCost = axis->getInteriorCost();
    dto.leafCost = axis->getLeafCost();
    dto.objectCost = axis->getObjectCost();
    return dto;
  }
  if (std::shared_ptr<SAHKDTreeFactory> sah =
        std::dynamic_pointer_cast<SAHKDTreeFactory>(kdtf)) {
    dto.factoryType = 2;
    dto.lossNodes = sah->getLossNodes();
    dto.interiorCost = sah->getInteriorCost();
    dto.leafCost = sah->getLeafCost();
    dto.objectCost = sah->getObjectCost();
    return dto;
  }

  dto.factoryType = 1;
  return dto;
}

void
applySAHDescriptor(std::shared_ptr<KDTreeFactory> const& kdtf,
                   KDTreeFactoryDescriptorDTO const& dto)
{
  if (std::shared_ptr<SAHKDTreeFactory> sah =
        std::dynamic_pointer_cast<SAHKDTreeFactory>(kdtf)) {
    sah->setLossNodes(dto.lossNodes);
    sah->setInteriorCost(dto.interiorCost);
    sah->setLeafCost(dto.leafCost);
    sah->setObjectCost(dto.objectCost);
    return;
  }

  if (std::shared_ptr<MultiThreadSAHKDTreeFactory> mtSah =
        std::dynamic_pointer_cast<MultiThreadSAHKDTreeFactory>(kdtf)) {
    std::shared_ptr<SAHKDTreeFactory> base =
      std::dynamic_pointer_cast<SAHKDTreeFactory>(mtSah->getKdtf());
    if (base != nullptr) {
      base->setLossNodes(dto.lossNodes);
      base->setInteriorCost(dto.interiorCost);
      base->setLeafCost(dto.leafCost);
      base->setObjectCost(dto.objectCost);
    }
  }
}

std::shared_ptr<KDTreeFactory>
factoryFromDTO(KDTreeFactoryDescriptorDTO const& dto)
{
  std::size_t const numJobs = std::max<std::size_t>(dto.numJobs, 1);
  std::size_t const geomJobs = std::max<std::size_t>(dto.geomJobs, 1);

  std::shared_ptr<KDTreeFactory> kdtf = nullptr;
  if (dto.factoryType == 1) {
    if (numJobs > 1)
      kdtf = KDTreeFactoryMaker::makeSimpleMultiThread(numJobs, geomJobs);
    else
      kdtf = KDTreeFactoryMaker::makeSimple();
  } else if (dto.factoryType == 2) {
    std::size_t const lossNodes = std::max<std::size_t>(dto.lossNodes, 1);
    if (numJobs > 1)
      kdtf =
        KDTreeFactoryMaker::makeSAHMultiThread(lossNodes, numJobs, geomJobs);
    else
      kdtf = KDTreeFactoryMaker::makeSAH(lossNodes);
  } else if (dto.factoryType == 3) {
    std::size_t const lossNodes = std::max<std::size_t>(dto.lossNodes, 1);
    if (numJobs > 1)
      kdtf = KDTreeFactoryMaker::makeAxisSAHMultiThread(
        lossNodes, numJobs, geomJobs);
    else
      kdtf = KDTreeFactoryMaker::makeAxisSAH(lossNodes);
  } else if (dto.factoryType == 4) {
    std::size_t const lossNodes = std::max<std::size_t>(dto.lossNodes, 1);
    if (numJobs > 1)
      kdtf = KDTreeFactoryMaker::makeFastSAHMultiThread(
        lossNodes, numJobs, geomJobs);
    else
      kdtf = KDTreeFactoryMaker::makeFastSAH(lossNodes);
  } else {
    throw HeliosException("Unexpected KDTree factory type while loading scene");
  }

  applySAHDescriptor(kdtf, dto);
  kdtf->setBuildingLightNodes(dto.buildLightNodes);
  if (std::shared_ptr<MultiThreadKDTreeFactory> mt =
        std::dynamic_pointer_cast<MultiThreadKDTreeFactory>(kdtf)) {
    mt->getKdtf()->setBuildingLightNodes(dto.buildLightNodes);
  }
  return kdtf;
}

void
nodeToDTO(
  LightKDTreeNode const* node,
  KDNodeDTO& dto,
  std::unordered_map<Primitive const*, std::size_t> const& primitiveIndexMap)
{
  dto.splitAxis = node->splitAxis;
  dto.splitPos = node->splitPos;

  dto.primitiveIndices.clear();
  if (node->primitives != nullptr) {
    dto.primitiveIndices.reserve(node->primitives->size());
    for (Primitive const* primitive : *node->primitives) {
      auto const it = primitiveIndexMap.find(primitive);
      if (it == primitiveIndexMap.end()) {
        throw HeliosException(
          "KDTree leaf references a primitive not present in scene");
      }
      dto.primitiveIndices.push_back((std::int64_t)it->second);
    }
  }

  if (node->left != nullptr) {
    dto.left = std::make_shared<KDNodeDTO>();
    nodeToDTO(node->left, *dto.left, primitiveIndexMap);
  }
  if (node->right != nullptr) {
    dto.right = std::make_shared<KDNodeDTO>();
    nodeToDTO(node->right, *dto.right, primitiveIndexMap);
  }
}

LightKDTreeNode*
nodeFromDTO(KDNodeDTO const& dto, std::vector<Primitive*>& scenePrimitives)
{
  LightKDTreeNode* node = new LightKDTreeNode();
  node->splitAxis = dto.splitAxis;
  node->splitPos = dto.splitPos;

  if (!dto.primitiveIndices.empty()) {
    node->primitives = std::make_shared<std::vector<Primitive*>>();
    node->primitives->reserve(dto.primitiveIndices.size());
    for (std::int64_t idx : dto.primitiveIndices) {
      if (idx < 0 || (std::size_t)idx >= scenePrimitives.size()) {
        delete node;
        throw HeliosException(
          "Invalid primitive index while restoring KDTree node");
      }
      node->primitives->push_back(scenePrimitives[(std::size_t)idx]);
    }
  }

  if (dto.left != nullptr)
    node->left = nodeFromDTO(*dto.left, scenePrimitives);
  if (dto.right != nullptr)
    node->right = nodeFromDTO(*dto.right, scenePrimitives);

  return node;
}

SceneDTO
sceneToDTO(Scene const& scene)
{
  SceneDTO dto;
  dto.assetId = scene.id;
  dto.assetName = scene.name;
  dto.sourceFilePath = scene.sourceFilePath;
  dto.defaultReflectance = scene.getDefaultReflectance();

  if (scene.getBBox() != nullptr) {
    dto.hasBBox = true;
    dto.bboxMin = toDTO(scene.getBBox()->getMin());
    dto.bboxMax = toDTO(scene.getBBox()->getMax());
  }
  if (scene.getBBoxCRS() != nullptr) {
    dto.hasBBoxCRS = true;
    dto.bboxCrsMin = toDTO(scene.getBBoxCRS()->getMin());
    dto.bboxCrsMax = toDTO(scene.getBBoxCRS()->getMax());
  }

  std::unordered_map<ScenePart const*, std::size_t> partIndexMap;
  partIndexMap.reserve(scene.parts.size());
  for (std::size_t i = 0; i < scene.parts.size(); ++i)
    partIndexMap[scene.parts[i].get()] = i;

  std::unordered_map<Material const*, int> materialIndexMap;
  std::unordered_map<Primitive const*, std::size_t> primitiveIndexMap;
  primitiveIndexMap.reserve(scene.primitives.size());

  dto.primitives.reserve(scene.primitives.size());
  for (std::size_t i = 0; i < scene.primitives.size(); ++i) {
    Primitive const* primitive = scene.primitives[i];
    primitiveIndexMap[primitive] = i;

    int materialIndex = -1;
    if (primitive->material != nullptr) {
      Material const* key = primitive->material.get();
      auto it = materialIndexMap.find(key);
      if (it == materialIndexMap.end()) {
        materialIndex = (int)dto.materials.size();
        materialIndexMap[key] = materialIndex;
        dto.materials.push_back(toDTO(*primitive->material));
      } else {
        materialIndex = it->second;
      }
    }

    int partIndex = -1;
    if (primitive->part != nullptr) {
      auto it = partIndexMap.find(primitive->part.get());
      if (it != partIndexMap.end())
        partIndex = (int)it->second;
    }

    dto.primitives.push_back(
      primitiveToDTO(primitive, partIndex, materialIndex));
  }

  dto.parts.reserve(scene.parts.size());
  for (std::shared_ptr<ScenePart> const& part : scene.parts) {
    dto.parts.push_back(partToDTO(part, primitiveIndexMap));
  }

  if (scene.getKDGroveFactory() != nullptr &&
      scene.getKDGroveFactory()->getKdtf() != nullptr) {
    dto.hasFactory = true;
    dto.factory = factoryToDTO(scene.getKDGroveFactory()->getKdtf());
  }

  std::shared_ptr<KDGrove> grove = scene.getKDGrove();
  if (grove != nullptr) {
    dto.hasKDGrove = true;
    KDGroveDTO groveDTO;
    std::size_t const numTrees = grove->getNumTrees();
    groveDTO.trees.reserve(numTrees);

    std::unordered_map<KDGroveSubject const*, int> subjectIndexMap;
    for (std::size_t i = 0; i < scene.parts.size(); ++i) {
      KDGroveSubject const* subject =
        dynamic_cast<KDGroveSubject const*>(scene.parts[i].get());
      if (subject != nullptr)
        subjectIndexMap[subject] = (int)i;
    }

    std::vector<BasicDynGroveSubject<GroveKDTreeRaycaster,
                                     DynMovingObject>*> const& subjects =
      grove->getSubjects();

    for (std::size_t i = 0; i < numTrees; ++i) {
      TreeDTO treeDTO;

      if (i < subjects.size() && subjects[i] != nullptr) {
        KDGroveSubject const* subject =
          static_cast<KDGroveSubject*>(subjects[i]);
        auto const it = subjectIndexMap.find(subject);
        if (it == subjectIndexMap.end()) {
          throw HeliosException(
            "Dynamic KDTree subject cannot be matched to scene parts");
        }
        treeDTO.subjectPartIndex = it->second;
      }

      std::shared_ptr<GroveKDTreeRaycaster> tree = grove->getTreeShared(i);
      KDTreeNodeRoot const* root =
        dynamic_cast<KDTreeNodeRoot const*>(tree->root.get());
      if (root != nullptr) {
        treeDTO.hasRoot = true;
        nodeToDTO(root, treeDTO.root, primitiveIndexMap);
        treeDTO.statsMaxPrimsInLeaf = root->stats_maxNumPrimsInLeaf;
        treeDTO.statsMinPrimsInLeaf = root->stats_minNumPrimsInLeaf;
        treeDTO.statsMaxDepth = root->stats_maxDepthReached;
        treeDTO.statsNumInterior = root->stats_numInterior;
        treeDTO.statsNumLeaves = root->stats_numLeaves;
        treeDTO.statsTotalCost = root->stats_totalCost;
      }

      groveDTO.trees.push_back(treeDTO);
    }
    dto.kdgrove = std::move(groveDTO);
  }

  return dto;
}

void
loadSceneFromDTO(Scene& scene, SceneDTO const& dto)
{
  scene.shutdown();

  scene.id = dto.assetId;
  scene.name = dto.assetName;
  scene.sourceFilePath = dto.sourceFilePath;
  scene.setDefaultReflectance(dto.defaultReflectance);

  scene.setBBox(dto.hasBBox ? std::make_shared<AABB>(fromDTO(dto.bboxMin),
                                                     fromDTO(dto.bboxMax))
                            : nullptr);
  scene.setBBoxCRS(
    dto.hasBBoxCRS
      ? std::make_shared<AABB>(fromDTO(dto.bboxCrsMin), fromDTO(dto.bboxCrsMax))
      : nullptr);

  std::vector<std::shared_ptr<Material>> materials;
  materials.reserve(dto.materials.size());
  for (MaterialDTO const& materialDTO : dto.materials)
    materials.push_back(fromDTO(materialDTO));

  scene.primitives.clear();
  scene.primitives.reserve(dto.primitives.size());
  for (PrimitiveDTO const& primitiveDTO : dto.primitives)
    scene.primitives.push_back(primitiveFromDTO(primitiveDTO, materials));

  std::vector<std::shared_ptr<ScenePart>> parts;
  parts.reserve(dto.parts.size());
  for (PartDTO const& partDTO : dto.parts)
    parts.push_back(partFromDTO(partDTO));

  for (std::size_t i = 0; i < dto.parts.size(); ++i) {
    PartDTO const& partDTO = dto.parts[i];
    std::shared_ptr<ScenePart> part = parts[i];
    part->mPrimitives.clear();
    part->mPrimitives.reserve(partDTO.primitiveIndices.size());
    for (std::size_t primitiveIndex : partDTO.primitiveIndices) {
      if (primitiveIndex >= scene.primitives.size())
        throw HeliosException(
          "Invalid scene part primitive index while loading");
      Primitive* primitive = scene.primitives[primitiveIndex];
      primitive->part = part;
      part->mPrimitives.push_back(primitive);
    }
  }

  for (std::size_t i = 0; i < dto.primitives.size(); ++i) {
    PrimitiveDTO const& primitiveDTO = dto.primitives[i];
    if (primitiveDTO.partIndex < 0)
      continue;
    if ((std::size_t)primitiveDTO.partIndex >= parts.size())
      throw HeliosException("Invalid primitive->part index while loading");
    if (scene.primitives[i]->part == nullptr)
      scene.primitives[i]->part = parts[(std::size_t)primitiveDTO.partIndex];
  }

  scene.parts = parts;

  if (dto.hasFactory) {
    std::shared_ptr<KDTreeFactory> kdtf = factoryFromDTO(dto.factory);
    scene.setKDGroveFactory(std::make_shared<KDGroveFactory>(kdtf));
  } else {
    scene.setKDGroveFactory(nullptr);
  }

  if (!dto.hasKDGrove) {
    scene.setKDGrove(nullptr);
    return;
  }

  std::shared_ptr<KDGrove> grove =
    std::make_shared<KDGrove>(dto.kdgrove.trees.size());

  std::shared_ptr<KDTreeFactory> sceneFactory = nullptr;
  if (scene.getKDGroveFactory() != nullptr)
    sceneFactory = scene.getKDGroveFactory()->getKdtf();

  for (TreeDTO const& treeDTO : dto.kdgrove.trees) {
    std::shared_ptr<LightKDTreeNode> root = nullptr;
    if (treeDTO.hasRoot) {
      KDTreeNodeRoot* rootRaw = new KDTreeNodeRoot();
      rootRaw->splitAxis = treeDTO.root.splitAxis;
      rootRaw->splitPos = treeDTO.root.splitPos;
      if (!treeDTO.root.primitiveIndices.empty()) {
        rootRaw->primitives = std::make_shared<std::vector<Primitive*>>();
        rootRaw->primitives->reserve(treeDTO.root.primitiveIndices.size());
        for (std::int64_t idx : treeDTO.root.primitiveIndices) {
          if (idx < 0 || (std::size_t)idx >= scene.primitives.size()) {
            delete rootRaw;
            throw HeliosException(
              "Invalid root primitive index while loading KDTree");
          }
          rootRaw->primitives->push_back(scene.primitives[(std::size_t)idx]);
        }
      }
      if (treeDTO.root.left != nullptr)
        rootRaw->left = nodeFromDTO(*treeDTO.root.left, scene.primitives);
      if (treeDTO.root.right != nullptr)
        rootRaw->right = nodeFromDTO(*treeDTO.root.right, scene.primitives);
      rootRaw->stats_maxNumPrimsInLeaf = treeDTO.statsMaxPrimsInLeaf;
      rootRaw->stats_minNumPrimsInLeaf = treeDTO.statsMinPrimsInLeaf;
      rootRaw->stats_maxDepthReached = treeDTO.statsMaxDepth;
      rootRaw->stats_numInterior = treeDTO.statsNumInterior;
      rootRaw->stats_numLeaves = treeDTO.statsNumLeaves;
      rootRaw->stats_totalCost = treeDTO.statsTotalCost;
      root = std::shared_ptr<LightKDTreeNode>(rootRaw);
    }

    std::shared_ptr<KDTreeFactory> treeFactory = nullptr;
    if (sceneFactory != nullptr)
      treeFactory = std::shared_ptr<KDTreeFactory>(sceneFactory->clone());

    std::shared_ptr<GroveKDTreeRaycaster> tree =
      std::make_shared<GroveKDTreeRaycaster>(root, treeFactory);

    if (treeDTO.subjectPartIndex >= 0) {
      std::size_t const partIdx = (std::size_t)treeDTO.subjectPartIndex;
      if (partIdx >= scene.parts.size())
        throw HeliosException("Invalid dynamic subject index while loading");
      KDGroveSubject* subject =
        dynamic_cast<KDGroveSubject*>(scene.parts[partIdx].get());
      if (subject == nullptr)
        throw HeliosException(
          "KDTree subject points to a non-dynamic scene part");
      subject->registerObserverGrove(grove);
      grove->addSubject(subject, tree);
    } else {
      grove->addSubject(nullptr, tree);
    }
  }

  scene.setKDGrove(grove);
}

} // namespace

void
Scene::saveCereal(std::string const& path, int const compressionLevel) const
{
  SceneDTO dto = sceneToDTO(*this);
  std::ofstream ofs(path, std::ios::binary | std::ios::trunc);
  if (!ofs.is_open()) {
    throw HeliosException("Cannot open scene serialization output file: " +
                          path);
  }

  int const normalizedLevel = normalizeCompressionLevel(compressionLevel);
  if (normalizedLevel == 0) {
    cereal::BinaryOutputArchive archive(ofs);
    archive(dto);
    return;
  }

  boost::iostreams::zlib_params zp(normalizedLevel);
  boost::iostreams::filtering_ostream compressedOut;
  compressedOut.push(boost::iostreams::zlib_compressor(zp));
  compressedOut.push(ofs);

  cereal::BinaryOutputArchive archive(compressedOut);
  archive(dto);
  compressedOut.reset();
}

void
Scene::loadCereal(std::string const& path)
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    throw HeliosException("Cannot open scene serialization input file: " +
                          path);
  }

  SceneDTO dto;
  if (isLikelyZlibStream(ifs)) {
    boost::iostreams::filtering_istream compressedIn;
    compressedIn.push(boost::iostreams::zlib_decompressor());
    compressedIn.push(ifs);
    cereal::BinaryInputArchive archive(compressedIn);
    archive(dto);
    compressedIn.reset();
  } else {
    cereal::BinaryInputArchive archive(ifs);
    archive(dto);
  }

  loadSceneFromDTO(*this, dto);
  raycaster = (kdgrove == nullptr)
                ? nullptr
                : std::make_shared<KDGroveRaycaster>(kdgrove);
}
