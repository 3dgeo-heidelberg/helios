import pyhelios
import time
import numpy as np


def callback(output=None):
    with pyhelios.PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE:
        pyhelios.PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE.notify()


if __name__ == "__main__":
    pyhelios.loggingVerbose()
    pyhelios.setDefaultRandomnessGeneratorSeed("123")

    # Use TLS scene (calls ScanningDevice.cpp)
    simBuilder = pyhelios.SimulationBuilder(
        "data/surveys/demo/tls_arbaro_demo.xml", "data", "output/"
    )
    simBuilder.setNumThreads(0)
    simBuilder.setLasOutput(True)
    simBuilder.setWriteWaveform(True)
    simBuilder.setCallback(callback)
    sim = simBuilder.build()

    print("Running simulation...")
    sim.start()
    output = sim.join()
    print("Simulation finished.")

    npMeasurements, npTrajectories = pyhelios.outputToNumpy(output)
    print("Numpy measurements shape:", np.shape(npMeasurements))
