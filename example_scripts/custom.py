import pyhelios
import time
import numpy as np


def callback(output=None):
    with pyhelios.PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE:
        pyhelios.PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE.notify()


if __name__ == "__main__":
    pyhelios.loggingVerbose()
    pyhelios.setDefaultRandomnessGeneratorSeed("123")

    simBuilder = pyhelios.SimulationBuilder(
        "data/surveys/toyblocks/als_toyblocks.xml",
        "assets/",
        "output/"
    )

    # Set simulation config
    simBuilder.setNumThreads(0)
    simBuilder.setCallbackFrequency(10)
    simBuilder.setFinalOutput(True)
    simBuilder.setWriteWaveform(True)
    simBuilder.setWritePulse(True)
    simBuilder.setRebuildScene(True)
    simBuilder.setCalcEchowidth(True)
    simBuilder.setFullwaveNoise(False)
    simBuilder.setPlatformNoiseDisabled(True)
    simBuilder.setLegNoiseDisabled(True)
    simBuilder.setZipOutput(False)
    simBuilder.setLasOutput(False)
    simBuilder.setCallback(callback)

    # Build + run simulation
    sim = simBuilder.build()
    sim.start()
    sim.join()

    try:
        with open("/tmp/helios_subray_mode.txt", "r") as f:
            print(f.read())
    except FileNotFoundError:
        print("toyblocks ALS")
