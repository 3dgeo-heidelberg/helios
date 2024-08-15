import pyhelios
from threading import Condition as CondVar

PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE = CondVar()


class SimulationBuild:
    """SimulationBuild represents a PyHelios simulation built through a
    SimulationBuilder.

    Attributes:
        sim -- The PyHeliosSimulation instance
    """

    # ---  CONSTRUCTOR  --- #
    # --------------------- #
    def __init__(
        self,
        surveyPath,
        assetsDir,
        outputDir,
        numThreads,
        lasOutput,
        las10,
        zipOutput,
        splitByChannel=False,
        copy=False,
        fixedGpsTimeStart="",
        kdtFactory=4,
        kdtJobs=0,
        kdtSAHLossNodes=32,
        parallelizationStrategy=1,
        chunkSize=32,
        warehouseFactor=4
    ):
        if copy:
            return

        self.sim = pyhelios.Simulation(
            surveyPath,
            assetsDir,
            outputDir,
            numThreads,
            lasOutput,
            las10,
            zipOutput,
            splitByChannel,
            kdtFactory,
            kdtJobs,
            kdtSAHLossNodes,
            parallelizationStrategy,
            chunkSize,
            warehouseFactor
        )
        self.sim.fixed_gps_time_start = fixedGpsTimeStart

    # ---  CONTROL METHODS  --- #
    # ------------------------- #
    def start(self):
        self.sim.start()

    def pause(self):
        self.sim.pause()

    def stop(self):
        self.sim.stop()

    def resume(self):
        self.sim.resume()

    def join(self):
        # Conditional variable necessary for callback mode
        with PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE:
            measurements, trajectories, outpath, outpaths, finished = self.sim.join()
            while not finished:
                PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE.wait()
                measurements, trajectories, outpath, outpaths, finished = self.sim.join()
        return (measurements, trajectories, outpath, outpaths, finished)

    # ---  C O P Y  --- #
    # ----------------- #
    def copy(self):
        """Do a copy of this SimulationBuild.

        Return:
            SimulationBuild which is a copy of current one
        """
        copySim = SimulationBuild(None, None, None, None, None, None, True)
        copySim.sim = self.sim.copy()
        return copySim

    # ---  GETTERS and SETTERS  --- #
    # ----------------------------- #
    def isStarted(self):
        return self.sim.is_started

    def isPaused(self):
        return self.sim.is_paused

    def isStopped(self):
        return self.sim.is_stopped

    def isFinished(self):
        return self.sim.is_finished

    def isRunning(self):
        return self.sim.is_running

    def getScanner(self):
        return self.sim.scanner
