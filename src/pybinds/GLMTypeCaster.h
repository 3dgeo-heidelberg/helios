#pragma once

#include <pybind11/pybind11.h>
#include <glm/glm.hpp>


namespace pybind11 { namespace detail {

    template <> struct type_caster<glm::dvec3> {
    public:
        PYBIND11_TYPE_CASTER(glm::dvec3, _("dvec3"));

        bool load(handle src, bool) {
            if (!src) return false;
            if (!pybind11::isinstance<pybind11::iterable>(src)) return false;

            pybind11::tuple t = pybind11::cast<pybind11::tuple>(src);
            if (t.size() != 3) return false;

            glm::dvec3 value;
            value.x = pybind11::cast<double>(t[0]);
            value.y = pybind11::cast<double>(t[1]);
            value.z = pybind11::cast<double>(t[2]);

            return true;
        }
       
        static handle cast(const glm::dvec3& src, return_value_policy, handle) {
            return pybind11::make_tuple(src.x, src.y, src.z).release();
        }
    };

}} 