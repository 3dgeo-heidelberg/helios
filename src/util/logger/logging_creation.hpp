#pragma once

#include <logger.hpp>
#include <logging_common.hpp>

/**
 * @brief Assist factory to create loggers (that derive from 'logger')
 * via function pointers.
 *
 *
 * This way it is possible to make custom logger that sends log messages
 * to who knows where
 */
using logger_creator = logger* (*)(const logging_config_t&);
