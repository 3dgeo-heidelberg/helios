from .pyheliostools_exception import PyHeliosToolsException
from .simulation_build import SimulationBuild
from collections import namedtuple
from math import isnan
import os
import time

SimulationBuilderRotateFilter = namedtuple(
    'SimulationBuilderRotateFilter',
    ['q0', 'q1', 'q2', 'q3', 'id']
)

SimulationBuilderScaleFilter = namedtuple(
    'SimulationBuilderScaleFilter',
    ['factor', 'id']
)

SimulationBuilderTranslateFilter = namedtuple(
    'SimulationBuilderTranslateFilter',
    ['x', 'y', 'z', 'id']
)


class SimulationBuilder:
    """SimulationBuilder can be used to generate a SimulationBuild that can
    be used by a SimulationHandler to manage the entire lifecycle of a
    Helios++ simulation through PyHelios

    Attributes:
        surveyPath -- Path to survey file
        assetsDir -- Path to assets directory
        outputDir -- Path to output directory
        numThreads -- Number of threads (0 means as many as possible)
        fixedGpsTimeStart -- If it is a non empty string, it specify a fixed
            GPS start time, either as posix timestamp or "YYYY-MM-DD hh:mm:ss"
        lasOutput -- LAS output format flag
        las10     -- LAS v1.0 output format flag
        zipOutput -- Zip output format flag (can be unzipped with Helios++)
        splitByChannel -- Each scanning channel to a different file
        kdtFactory -- The KDT factory to be used (Simple, SAH; AxisSAH, FSAH)
        kdtJobs -- Number of threads to build KDT
        kdtSAHLossNodes -- Size of iterative SAH computation or approximation
        parallelizationStrategy -- Either chunk based or warehosue based
        chunkSize -- Defualt chunk size
        warehouseFactor -- As many tasks as factor times number of threads
        callbackFrequency -- Simulation control frequency (do not mismatch with
            the simulation operating frequency)
        finalOutput -- Final output (the one obtained after .join) flag
        legNoiseDisabled -- Leg noise disabling flag
        rebuildScene -- Rebuild scene enabling flag
        writeWaveform -- Write waveform enabling flag
        calcEchowidth -- Compute echowidth enabling flag
        fullwaveNoise -- Compute fullwave noise enabling flag
        platformNoiseDisabled -- Platform noise disabling flag
        exportToFile -- Export to file enabling flag
        callback -- Callback function to be used when callbackFrequency is
            greater than 0. It can be None, then no callbacks will occur
        rotateFilters -- List of rotate filters to apply
        scaleFilters --  List of scale filters to apply
        translateFilters -- List of translate filters to apply
    """

    # ---  CONSTRUCTOR  --- #
    # --------------------- #
    def __init__(self, surveyPath, assetsDir, outputDir):
        # Base values
        self.setSurveyPath(surveyPath)
        self.setAssetsDir(assetsDir)
        self.setOutputDir(outputDir)

        # Default values
        self.makeDefault()

    # ---  D E F A U L T  --- #
    # ----------------------- #
    def makeDefault(self):
        self.setNumThreads(0)
        self.setFixedGpsTimeStart("")
        self.setLasOutput(False)
        self.setLas10(False)
        self.setZipOutput(False)
        self.setSplitByChannel(False)
        self.setKDTFactory(4)
        self.setKDTJobs(0)
        self.setKDTSAHLossNodes(32)
        self.setParallelizationStrategy(1)
        self.setChunkSize(32)
        self.setWarehouseFactor(4)
        self.setCallbackFrequency(0)
        self.setFinalOutput(True)
        self.setLegNoiseDisabled(True)
        self.setRebuildScene(False)
        self.setWriteWaveform(False)
        self.setCalcEchowidth(False)
        self.setFullwaveNoise(False)
        self.setPlatformNoiseDisabled(True)
        self.setExportToFile(True)
        self.setCallback(None)
        self.rotateFilters = []
        self.scaleFilters = []
        self.translateFilters = []

    # ---  BUILD METHOD  --- #
    # ---------------------- #
    def build(self):
        print('SimulationBuilder is building simulation ...')
        start = time.perf_counter()
        build = SimulationBuild(
            self.surveyPath,
            self.assetsDir,
            self.outputDir,
            self.numThreads,
            self.lasOutput,
            self.las10,
            self.zipOutput,
            self.splitByChannel,
            fixedGpsTimeStart=self.fixedGpsTimeStart
        )
        build.sim.callbackFrequency = self.callbackFrequency
        build.sim.finalOutput = self.finalOutput
        build.sim.exportToFile = self.exportToFile
        build.sim.loadSurvey(
            self.legNoiseDisabled,
            self.rebuildScene,
            self.writeWaveform,
            self.calcEchowidth,
            self.fullwaveNoise,
            self.platformNoiseDisabled
        )
        if self.callback is not None:
            build.sim.setCallback(self.callback)
        for rotateFilter in self.rotateFilters:
            build.sim.addRotateFilter(
                rotateFilter.q0,
                rotateFilter.q1,
                rotateFilter.q2,
                rotateFilter.q3,
                rotateFilter.id
            )
        for scaleFilter in self.scaleFilters:
            build.sim.addScaleFilter(
                scaleFilter.factor,
                scaleFilter.id
            )
        for translateFilter in self.translateFilters:
            build.sim.addTranslateFilter(
                translateFilter.x,
                translateFilter.y,
                translateFilter.z,
                translateFilter.id
            )

        end = time.perf_counter()
        print(
            'SimulationBuilder built simulation in {t} seconds'
            .format(
                t=end - start
            )
        )
        return build

    # ---  GETTERS and SETTERS  --- #
    # ----------------------------- #
    def setSurveyPath(self, surveyPath):
        self.validatePath(surveyPath)
        self.surveyPath = surveyPath

    def setAssetsDir(self, assetsDir):
        self.validateDir(assetsDir)
        self.assetsDir = assetsDir

    def setOutputDir(self, outputDir):
        # self.validateDir(outputDir)  # note: the output dir does not have to exist at this point
        #                                      as it can be created by (py)helios when running it
        self.outputDir = outputDir

    def setNumThreads(self, numThreads):
        self.validateNumThreads(numThreads)
        self.numThreads = numThreads

    def setFixedGpsTimeStart(self, fixedGpsTimeStart):
        self.validateFixedGpsTimeStart(fixedGpsTimeStart)
        self.fixedGpsTimeStart = fixedGpsTimeStart

    def setLasOutput(self, lasOutput):
        self.validateBoolean(lasOutput)
        self.lasOutput = lasOutput

    def setLas10(self, las10):
        self.validateBoolean(las10)
        self.las10 = las10

    def setZipOutput(self, zipOutput):
        self.validateBoolean(zipOutput)
        self.zipOutput = zipOutput

    def setSplitByChannel(self, splitByChannel):
        self.validateBoolean(splitByChannel)
        self.splitByChannel = splitByChannel

    def setKDTFactory(self, kdtFactory):
        self.validateKDTFactory(kdtFactory)
        self.kdtFactory = kdtFactory

    def setKDTJobs(self, kdtJobs):
        self.validateNumThreads(kdtJobs)
        self.kdtJobs = kdtJobs

    def setKDTSAHLossNodes(self, sahLossNodes):
        self.validateNonNegativeInteger(sahLossNodes)
        self.sahLossNodes = sahLossNodes

    def setParallelizationStrategy(self, parallelizationStrategy):
        self.validateParallelizationStrategy(parallelizationStrategy)
        self.parallelizationStrategy = parallelizationStrategy

    def setChunkSize(self, chunkSize):
        self.validateInteger(chunkSize)
        self.chunkSize = chunkSize

    def setWarehouseFactor(self, warehouseFactor):
        self.validateNonNegativeInteger(warehouseFactor)
        self.warehouseFactor = warehouseFactor

    def setCallbackFrequency(self, freq):
        self.validateCallbackFrequency(freq)
        self.callbackFrequency = freq

    def setFinalOutput(self, finalOutput):
        self.validateBoolean(finalOutput)
        self.finalOutput = finalOutput

    def setLegNoiseDisabled(self, legNoiseDisabled):
        self.validateBoolean(legNoiseDisabled)
        self.legNoiseDisabled = legNoiseDisabled

    def setRebuildScene(self, rebuildScene):
        self.validateBoolean(rebuildScene)
        self.rebuildScene = rebuildScene

    def setWriteWaveform(self, writeWaveform):
        self.validateBoolean(writeWaveform)
        self.writeWaveform = writeWaveform

    def setCalcEchowidth(self, calcEchowidth):
        self.validateBoolean(calcEchowidth)
        self.calcEchowidth = calcEchowidth

    def setFullwaveNoise(self, fullwaveNoise):
        self.validateBoolean(fullwaveNoise)
        self.fullwaveNoise = fullwaveNoise

    def setPlatformNoiseDisabled(self, platformNoiseDisabled):
        self.validateBoolean(platformNoiseDisabled)
        self.platformNoiseDisabled = platformNoiseDisabled

    def setExportToFile(self, exportToFile):
        self.validateBoolean(exportToFile)
        self.exportToFile = exportToFile

    def setCallback(self, callback):
        self.validateCallback(callback)
        self.callback = callback

    def addRotateFilter(self, q0, q1, q2, q3, id):
        self.rotateFilters.append(SimulationBuilderRotateFilter(
            q0, q1, q2, q3, id
        ))

    def addScaleFilter(self, factor, id):
        self.scaleFilters.append(SimulationBuilderScaleFilter(factor, id))

    def addTranslateFilter(self, x, y, z, id):
        self.translateFilters.append(SimulationBuilderTranslateFilter(
            x, y, z, id
        ))

    # ---  VALIDATION METHODS  --- #
    # ---------------------------- #
    def validatePath(self, path):
        if not os.path.exists(path):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Path \'{p}\' does not exist'.format(p=path)
            )

        if not os.path.isfile(path):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Path \'{p}\' does not point to a file'.format(p=path)
            )

        if not os.access(path, os.R_OK):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Cannot read path \'{p}\''.format(p=path)
            )

    def validateDir(self, dir):
        if not os.path.exists(dir):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Dir \'{d}\' does not exist'.format(d=dir)
            )

        if not os.path.isdir(dir):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Dir \'{d}\' does not point to a directory'.format(d=dir)
            )

        if not (os.access(dir, os.R_OK) and os.access(dir, os.X_OK)):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Cannot access directory \'{d}\''.format(d=dir)
            )

    def validateNumThreads(self, numThreads):
        if isnan(numThreads):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'NaN number of threads is not allowed'
            )

        if numThreads < 0:
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Number of threads {n} < 0 is not allowed'.format(
                    n=numThreads
                )
            )

    def validateFixedGpsTimeStart(self, fgts):
        if fgts == '':
            return
        if fgts.isnumeric():
            return
        if len(fgts) == 19 and \
                fgts[0:4].isnumeric() and \
                fgts[4] == '-' and \
                fgts[5:7].isnumeric() and \
                fgts[7] == '-' and \
                fgts[8:10].isnumeric() and \
                fgts[10] == ' ' and \
                fgts[11:13].isnumeric() and \
                fgts[13] == ':' and \
                fgts[14:16].isnumeric() and \
                fgts[16] == ':' and \
                fgts[17:].isnumeric():
            return
        raise PyHeliosToolsException(
            'SimulationBuilder EXCEPTION!\n\t'
            'Given fixed GPS time start "{fgts}" is not allowed'.format(
                fgts=fgts
            )
        )

    def validateCallbackFrequency(self, freq):
        if isnan(freq):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'NaN frequency is not allowed'
            )

        if freq < 0:
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Frequency {f} < 0 is not allowed'.format(
                    f=freq
                )
            )

    def validateBoolean(self, b):
        if type(b) != bool:
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                '{b} is not a boolean'.format(b=b)
            )

    def validateInteger(self, i):
        if type(i) != int:
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                '{i} is not integer'.format(i=i)
            )

    def validateNonNegativeInteger(self, i):
        self.validateInteger(i)
        if i < 0:
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                '{i} is not a non negative integer'.format(i=i)
            )

    def validateCallback(self, callback):
        if callback is None:
            return

        if not hasattr(callback, '__call__'):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Callback is NOT callable but it MUST be callable'
            )

    def validateKDTFactory(self, kdtFactory):
        if kdtFactory not in (1, 2, 3, 4):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Unexpected KDT Factory {kdtf}'.format(kdtf=kdtFactory)
            )

    def validateParallelizationStrategy(self, parallelizationStrategy):
        if parallelizationStrategy not in (0, 1):
            raise PyHeliosToolsException(
                'SimulationBuilder EXCEPTION!\n\t'
                'Unexpected parallelization strategy {ps}'
                .format(ps=parallelizationStrategy)
            )
