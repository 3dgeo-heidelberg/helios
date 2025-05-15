#include "Vertex.h"

// ***  CONSTRUCTION / DESTRUCTION  *** //
// ************************************ //
Vertex::Vertex(const Vertex& v)
{
  this->pos = glm::dvec3(v.pos);
  this->normal = glm::dvec3(v.normal);
  this->color = Color4f(v.color);
  this->texcoords = glm::dvec2(v.texcoords);
}

// ***  M E T H O D S  *** //
// *********************** //
double*
Vertex::matxvec(double** mat, double* vec)
{
  double* res = new double[3];

  for (int i = 0; i < 3; i++) {
    double tmp = 0;
    for (int j = 0; j < 3; j++) {
      tmp += mat[i][j] * vec[j];
    }
    res[i] = tmp;
  }

  return res;
}

Vertex
Vertex::rotateVertex(Vertex v, double** rotationMatrix)
{
  double vector[] = { v.pos.x, v.pos.y, v.pos.z };
  double* result = matxvec(rotationMatrix, vector);
  Vertex newVert = Vertex();
  newVert.pos = glm::dvec3(result[0], result[1], result[2]);

  return newVert;
}

std::ostream&
operator<<(std::ostream& out, Vertex* v)
{
  out << "(" << v->getX() << ", " << v->getY() << ", " << v->getZ() << ")";
  return out;
}
