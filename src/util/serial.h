#pragma once

#include "Triangle.h"

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, glm::dvec2 & vec, const unsigned int version)
		{
			ar & vec.x;
			ar & vec.y;
		}

		template<class Archive>
		void serialize(Archive & ar, glm::dvec3 & vec, const unsigned int version)
		{
			ar & vec.x;
			ar & vec.y;
			ar & vec.z;
		}

		template<class Archive>
		void save_construct_data(Archive & ar, const Triangle * t, const unsigned int file_version) {
			// save data required to construct instance
			ar << t->verts;
		}
		template<class Archive>
		void load_construct_data(Archive & ar, Triangle * t, const unsigned int file_version) {
			// retrieve data from archive required to construct new instance
			Vertex verts[3];
			ar >> verts;
			// invoke inplace constructor to initialize instance of my_class
			::new(t)Triangle(verts[0], verts[1], verts[2]);
		}
	}
}
