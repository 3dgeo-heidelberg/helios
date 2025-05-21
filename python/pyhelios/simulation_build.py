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
        warehouseFactor=4,
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
            warehouseFactor,
        )
        self.sim.fixedGpsTimeStart = fixedGpsTimeStart

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
            output = self.sim.join()
            while not output.finished:
                PYHELIOS_SIMULATION_BUILD_CONDITION_VARIABLE.wait()
                output = self.sim.join()
        return output

    # ---  C O P Y  --- #
    # ----------------- #
    def copy(self):
        """Do a copy of this SimulationBuild.

        Return:
            SimulationBuild which is a copy of current one
        """
        copySim = SimulationBuild(None, None, None, None, None, None, None, None, True)
        copySim.sim = self.sim.copy()
        return copySim

    # ---  GETTERS and SETTERS  --- #
    # ----------------------------- #
    def isStarted(self):
        return self.sim.isStarted()

    def isPaused(self):
        return self.sim.isPaused()

    def isStopped(self):
        return self.sim.isStopped()

    def isFinished(self):
        return self.sim.isFinished()

    def isRunning(self):
        return self.sim.isRunning()

    def getScanner(self):
        return self.sim.getScanner()
