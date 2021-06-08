#pragma once

#include <string>

/**
 * @brief Lidar simulation class
 *
 * It serves as entry point for Helios++ execution
 */
class LidarSim {
public:
    /**
     * @brief Initialize a LiDAR simulation
     * @param surveyPath Path to the survey file
     * @param assetsPath Path to the assets directory
     * @param outputPath  Path to the output directory
     * @param writeWaveform Write wave form flag. True to enable write wave
     *  form, false otherwise
     * @param calcEchowidth Calc echo width flag. True to enable echo width
     *  computation, false otherwise
     * @param njobs Number of concurrent jobs (0 means as many as possible)
     * @param fullWaveNoise Flag to specify full wave noise usage. True to
     *  enable full wave noise, false otherwise
     * @param platformNoiseDisabled Flag to specify platform noise disabled.
     *  True means platform noise is disabled, false means it is enabled
     * @param legNoiseDisabled Flag to specify leg noise disabled.
     *  True means leg noise is disabled, false means it is enabled
     * @param rebuildScene Flag to specify rebuild scene policy. True means
     *  scene will be build even when a previously built scene has been found,
     *  false means previously built scene will be used when available
     * @param lasOutput Flag to specify LAS output format. True implies using
     *  LAS output format, false implies don't
     * @param las10 Flag to specify that the output format mmust be LAS v1.0.
     * Used along with lasOutput
     * @param zipOutput Flag to specify output zipping. True implies output
     *  will be zipped, false means it will not
     * @param fixedIncidenceAngle Flag to specify usage of fixed incidence
     *  angle. True means fixed incidence angle will be used, false implies
     *  it will not
     * @param lasScale Specify LAS format scale factor
     */
    void init(
        std::string surveyPath,
        std::string assetsPath,
        std::string outputPath,
        bool writeWaveform = false,
        bool calcEchowidth = false,
        size_t njobs = 0,
        bool fullWaveNoise = false,
        bool platformNoiseDisabled = false,
        bool legNoiseDisabled = false,
        bool rebuildScene = false,
        bool lasOutput = false,
        bool las10 = false,
        bool zipOutput = false,
        bool fixedIncidenceAngle = false,
        double lasScale = 0.0001
    );
};