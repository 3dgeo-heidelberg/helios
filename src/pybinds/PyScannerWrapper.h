#pragma once

#ifdef PYTHON_BINDING

#include <PyBeamDeflectorWrapper.h>
namespace pyhelios{ class PyDetectorWrapper;};
#include <PyIntegerList.h>
#include <PyNoiseSourceWrapper.h>
#include <PyRandomnessGeneratorWrapper.h>
#include <PyDoubleVector.h>
using pyhelios::PyBeamDeflectorWrapper;
using pyhelios::PyDetectorWrapper;
using pyhelios::PyIntegerList;
using pyhelios::PyNoiseSourceWrapper;
using pyhelios::PyRandomnessGeneratorWrapper;
using pyhelios::PyDoubleVector;

#include <scanner/Scanner.h>
#include <maths/WaveMaths.h>

namespace pyhelios{

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for Scanner class.
 */
class PyScannerWrapper{
public:
    // ***  ATTRIBUTES  *** //
    // ******************** //
    Scanner &scanner;

    // ***  CONSTRUCTION / DESTRUCTION  *** //
    // ************************************ //
    PyScannerWrapper(Scanner &scanner) : scanner(scanner) {}
    virtual ~PyScannerWrapper() = default;

    // ***  M E T H O D S  *** //
    // *********************** //
    void initializeSequentialGenerators(){
        scanner.initializeSequentialGenerators();
    }
    void buildScanningPulseProcess(
        int const parallelizationStrategy,
        PulseTaskDropper &dropper,
        std::shared_ptr<PulseThreadPoolInterface> pool
    ){
        scanner.buildScanningPulseProcess(
            parallelizationStrategy,
            dropper,
            pool
        );
    }
    void applySettings(std::shared_ptr<ScannerSettings> settings){
        scanner.applySettings(settings);
    }
    std::shared_ptr<ScannerSettings> retrieveCurrentSettings(){
        return scanner.retrieveCurrentSettings();
    }
    void applySettingsFWF(FWFSettings settings){
        scanner.applySettingsFWF(settings);
    }
    void doSimStep(
        unsigned int legIndex,
        double currentGpsTime
    ){
        scanner.doSimStep(legIndex, currentGpsTime);
    }
    std::string toString(){
        return scanner.toString();
    }
    void calcRaysNumber(){
        scanner.calcRaysNumber();
    }
    void prepareDiscretization(){
        scanner.prepareDiscretization();
    }
    int calcTimePropagation(
        std::vector<double> & timeWave, int const numBins
    ){
        return WaveMaths::calcPropagationTimeLegacy(
            timeWave,
            numBins,
            scanner.getFWFSettings(0).binSize_ns,
            scanner.getPulseLength_ns(0),
            7.0  // 3.5 too many ops., 7.0 just one op.
        );
    }
    double calcFootprintArea(double const distance) const{
        return scanner.calcFootprintArea(distance);
    }
    double calcFootprintRadius(double distance){
        return scanner.calcFootprintRadius(distance);
    }
    double calcAtmosphericAttenuation() const{
        return scanner.calcAtmosphericAttenuation();
    }
    Rotation calcAbsoluteBeamAttitude() const{
        return scanner.calcAbsoluteBeamAttitude();
    }
    inline bool checkMaxNOR(int nor) {return scanner.checkMaxNOR(nor);}


    // ***  SIM STEP UTILS  *** //
    // ************************ //
    void handleSimStepNoise(
        glm::dvec3 & absoluteBeamOrigin,
        Rotation & absoluteBeamAttitude
    ){
        scanner.handleSimStepNoise(absoluteBeamOrigin, absoluteBeamAttitude);
    }
    inline void onLegComplete() {scanner.onLegComplete();}
    void inline onSimulationFinished() {scanner.onSimulationFinished();}
    void handleTrajectoryOutput(double const currentGpsTime){
        scanner.handleTrajectoryOutput(currentGpsTime);
    }
    void trackOutputPath(std::string const &path){
        scanner.trackOutputPath(path);
    }

    // *** GETTERs and SETTERs *** //
    // *************************** //
    inline int getCurrentPulseNumber() const {
        return scanner.getCurrentPulseNumber();
    }
    inline int getNumRays() const {
        return scanner.getNumRays();
    }
    inline void setNumRays(int const numRays) {
        scanner.setNumRays(numRays);
    }
    inline int getPulseFreq_Hz() const {
        return scanner.getPulseFreq_Hz();
    }
    void setPulseFreq_Hz(int const pulseFreq_Hz){
        scanner.setPulseFreq_Hz(pulseFreq_Hz);
    }
    double getPulseLength_ns(size_t const idx){
        return scanner.getPulseLength_ns(idx);
    }
    inline double getPulseLength_ns() const {
        return scanner.getPulseLength_ns(0);
    }
    void setPulseLength_ns(
        double const pulseLength_ns, size_t const idx
    ){
        scanner.setPulseLength_ns(pulseLength_ns, idx);
    }
    inline void setPulseLength_ns(double const pulseLength_ns){
        scanner.setPulseLength_ns(pulseLength_ns, 0);
    }
    inline bool lastPulseWasHit() const {
        return scanner.lastPulseWasHit();
    }
    void setLastPulseWasHit(bool lastPulseWasHit){
        scanner.setLastPulseWasHit(lastPulseWasHit);
    }
    double getBeamDivergence(size_t const idx) const{
        return scanner.getBeamDivergence(idx);
    }
    inline double getBeamDivergence() const {
        return scanner.getBeamDivergence(0);
    }
    void setBeamDivergence(
        double const beamDivergence, size_t const idx
    ){
        scanner.setBeamDivergence(beamDivergence, idx);
    }
    inline void setBeamDivergence(double const beamDivergence){
        scanner.setBeamDivergence(beamDivergence, 0);
    }
    double getAveragePower(size_t const idx) const{
        return scanner.getAveragePower(idx);
    }
    inline double getAveragePower() const {
        return scanner.getAveragePower(0);
    }
    void setAveragePower(
        double const averagePower, size_t const idx
    ){
        scanner.setAveragePower(averagePower, idx);
    }
    inline void setAveragePower(double const averagePower){
        scanner.setAveragePower(averagePower, 0);
    }
    double getBeamQuality(size_t const idx) const{
        return scanner.getBeamQuality(idx);
    }
    inline double getBeamQuality() const {
        return scanner.getBeamQuality(0);
    }
    void setBeamQuality(
        double const beamQuality, size_t const idx
    ){
        scanner.setBeamQuality(beamQuality, idx);
    }
    inline void setBeamQuality(double const beamQuality){
        scanner.setBeamQuality(beamQuality, 0);
    }
    double getEfficiency(size_t const idx) const{
        return scanner.getEfficiency(idx);
    }
    inline double getEfficiency() const {
        return scanner.getEfficiency(0);
    }
    void setEfficiency(double const efficiency, size_t const idx=0){
        scanner.setEfficiency(efficiency, idx);
    }
    inline void setEfficiency(double const efficiency){
        scanner.setEfficiency(efficiency, 0);
    }
    double getReceiverDiameter(size_t const idx) const{
        return scanner.getReceiverDiameter(idx);
    }
    inline double getReceiverDiameter() const {
        return scanner.getReceiverDiameter(0);
    }
    void setReceiverDiameter(
        double const receiverDiameter, size_t const idx
    ){
        scanner.setReceiverDiameter(receiverDiameter, idx);
    }
    inline void setReceiverDiameter(double const receiverDiameter)
    {setReceiverDiameter(receiverDiameter, 0);}
    double getVisibility(size_t const idx) const{
        return scanner.getVisibility(idx);
    }
    inline double getVisibility() const{return scanner.getVisibility(0);}
    void setVisibility(double const visibility, size_t const idx){
        scanner.setVisibility(visibility, idx);
    }
    inline void setVisibility(double const visibility)
    {setVisibility(visibility, 0);}
    double getWavelength(size_t const idx) const{
        return scanner.getWavelength();
    }
    inline double getWavelength() const {
        return scanner.getWavelength(0);
    }
    void setWavelength(double const wavelength, size_t const idx){
        scanner.setWavelength(wavelength, idx);
    }
    inline void setWavelength(double const wavelength){
        scanner.setWavelength(wavelength, 0);
    }
    double getAtmosphericExtinction(size_t const idx) const{
        return scanner.getAtmosphericExtinction(idx);
    }
    inline double getAtmosphericExtinction() const{
        return scanner.getAtmosphericExtinction(0);
    }
    void setAtmosphericExtinction(
        double const atmosphericExtinction,
        size_t const idx
    ){
        scanner.setAtmosphericExtinction(atmosphericExtinction, idx);
    }
    inline void setAtmosphericExtinction(double const atmosphericExtinction){
        scanner.setAtmosphericExtinction(atmosphericExtinction, 0);
    }
    double getBeamWaistRadius(size_t const idx) const{
        return scanner.getBeamWaistRadius(idx);
    }
    inline double getBeamWaistRadius() const {
        return scanner.getBeamWaistRadius(0);
    }
    void setBeamWaistRadius(
        double const beamWaistRadius, size_t const idx
    ){
        return scanner.setBeamWaistRadius(beamWaistRadius, idx);
    }
    inline void setBeamWaistRadius(double const beamWaistRadius){
        scanner.setBeamWaistRadius(beamWaistRadius, 0);
    }
    glm::dvec3 getHeadRelativeEmitterPosition(
        size_t const idx
    ) const{
        return scanner.getHeadRelativeEmitterPosition(idx);
    }
    void setHeadRelativeEmitterPosition(
        glm::dvec3 const &pos, size_t const idx
    ){
        scanner.setHeadRelativeEmitterPosition(pos, idx);
    }
    Rotation getHeadRelativeEmitterAttitude(size_t const idx) const{
        return scanner.getHeadRelativeEmitterAttitude(idx);
    }
    void setHeadRelativeEmitterAttitude(
        Rotation const &attitude, size_t const idx
    ){
        scanner.setHeadRelativeEmitterAttitude(attitude, idx);
    }
    double getBt2(size_t const idx) const{
        return scanner.getBt2(idx);
    }
    inline double getBt2() const {
        return scanner.getBt2(0);
    }
    void setBt2(double const bt2, size_t const idx){
        scanner.setBt2(bt2, idx);
    }
    inline void setBt2(double const bt2) {
        scanner.setBt2(bt2, 0);
    }
    double getDr2(size_t const idx) const{
        return scanner.getDr2(idx);
    }
    inline double getDr2() const {
        return scanner.getDr2(0);
    }
    void setDr2(double const dr2, size_t const idx){
        scanner.setDr2(dr2, idx);
    }
    inline void setDr2(double const dr2) {
        scanner.setDr2(dr2, 0);
    }
    inline bool isActive() const {
        return scanner.isActive();
    }
    inline void setActive(bool const active) {
        scanner.setActive(active);
    }
    inline bool isWriteWaveform() const {
        return scanner.isWriteWaveform();
    }
    inline void setWriteWaveform(bool const writeWaveform){
        scanner.setWriteWaveform(writeWaveform);
    }
    inline bool isCalcEchowidth() const {
        return scanner.isCalcEchowidth();
    }
    inline void setCalcEchowidth(bool const calcEchowidth){
        scanner.setCalcEchowidth(calcEchowidth);
    }
    inline bool isFullWaveNoise() const {
        return scanner.isFullWaveNoise();
    }
    inline void setFullWaveNoise(bool const fullWaveNoise){
        scanner.setFullWaveNoise(fullWaveNoise);
    }
    inline bool isPlatformNoiseDisabled() {
        return scanner.isPlatformNoiseDisabled();
    }
    inline void setPlatformNoiseDisabled(bool const platformNoiseDisabled){
        scanner.setPlatformNoiseDisabled(platformNoiseDisabled);
    }
    inline bool isFixedIncidenceAngle() const {
        return scanner.isFixedIncidenceAngle();
    }
    inline void setFixedIncidenceAngle(bool const fixedIncidenceAngle){
        scanner.setFixedIncidenceAngle(fixedIncidenceAngle);
    }
    inline std::string getScannerId() const {
        return scanner.getScannerId();
    }
    inline void setScannerId(std::string const &id) {
        scanner.setScannerId(id);
    }
    std::string getDeviceId(size_t const idx) const{
        return scanner.getDeviceId(idx);
    }
    inline std::string getDeviceId() const {
        return scanner.getDeviceId(0);
    }
    void setDeviceId(std::string const deviceId, size_t const idx){
        scanner.setDeviceId(deviceId, idx);
    }
    inline void setDeviceId(std::string const deviceId){
        scanner.setDeviceId(deviceId, 0);
    }
    size_t getNumDevices(){
        return scanner.getNumDevices();
    }

    // ***  PyScannerWrapper ADHOC  *** //
    // ******************************** //
    ScannerHead & getScannerHead(){return *scanner.getScannerHead();}
    PyBeamDeflectorWrapper * getPyBeamDeflector()
    {return new PyBeamDeflectorWrapper(scanner.getBeamDeflector());}
    PyDetectorWrapper * getPyDetectorWrapper();
    PyIntegerList * getSupportedPulseFrequencies()
    {return new PyIntegerList(scanner.getSupportedPulseFreqs_Hz());}
    Rotation & getRelativeAttitudeByReference(size_t const idx){
        return scanner.getHeadRelativeEmitterAttitudeByRef(idx);
    }
    Rotation & getRelativeAttitudeByReference(){
        return scanner.getHeadRelativeEmitterAttitudeByRef(0);
    }
    PythonDVec3 * getRelativePosition(size_t const idx){
        return new PythonDVec3(
            scanner.getHeadRelativeEmitterPositionByRef(idx)
        );
    }
    PythonDVec3 * getRelativePosition(){
        return new PythonDVec3(scanner.getHeadRelativeEmitterPositionByRef(0));
    }
    PyNoiseSourceWrapper * getIntersectionHandlingNoiseSource(){
        if(scanner.intersectionHandlingNoiseSource == nullptr) return nullptr;
        return new PyNoiseSourceWrapper(
            *scanner.intersectionHandlingNoiseSource
        );
    }
    PyRandomnessGeneratorWrapper * getRandGen1(){
        if(scanner.randGen1 == nullptr) return nullptr;
        return new PyRandomnessGeneratorWrapper(*scanner.randGen1);
    }
    PyRandomnessGeneratorWrapper * getRandGen2(){
        if(scanner.randGen2 == nullptr) return nullptr;
        return new PyRandomnessGeneratorWrapper(*scanner.randGen2);
    }
    PyDoubleVector * getTimeWave(){
        return new PyDoubleVector(scanner.getTimeWave());
    }
    FWFSettings getFWFSettings() {return scanner.getFWFSettings();}
    void setFWFSettings(FWFSettings const &fwfSettings)
    {scanner.setFWFSettings(fwfSettings);}
    int getNumTimeBins() {return scanner.getNumTimeBins();}
    void setNumTimeBins(int const numTimeBins)
    {scanner.setNumTimeBins(numTimeBins);}
    int getPeakIntensityIndex() {return scanner.getPeakIntensityIndex();}
    void setPeakIntensityIndex(int const peakIntensityIndex)
    {scanner.setPeakIntensityIndex(peakIntensityIndex);}
    double getTrajectoryTimeInterval()
    {return scanner.trajectoryTimeInterval_ns;}
    void setTrajectoryTimeInterval(double const trajectoryTimeInterval_ns){
        scanner.trajectoryTimeInterval_ns = trajectoryTimeInterval_ns;
    }

};

}

#endif