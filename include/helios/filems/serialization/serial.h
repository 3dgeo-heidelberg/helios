#pragma once

// This works around a known issue in boost:
// https://github.com/boostorg/serialization/issues/315
#ifdef BOOST_NO_EXCEPTIONS
#include <boost/throw_exception.hpp>
#endif
#include <boost/serialization/serialization.hpp>

/**
 * Here all serialization implementations are included in a single file.
 *
 * Developers working on this are strongly encouraged to read this brief
 *  tutorial:
 * https://www.boost.org/doc/libs/1_76_0/libs/serialization/doc/index.html
 */
#include <helios/filems/serialization/serial_adt.h>
#include <helios/filems/serialization/serial_arma.h>
#include <helios/filems/serialization/serial_dynamic.h>
#include <helios/filems/serialization/serial_glm.h>
#include <helios/filems/serialization/serial_primitives.h>
