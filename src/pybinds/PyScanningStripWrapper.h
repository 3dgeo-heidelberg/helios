#pragma once

#include <ScanningStrip.h>

#include <memory>

namespace pyhelios {

/**
 * @author Alberto M. Esmoris Pena
 * @version 1.0
 * @brief Wrapper for ScanningStrip class
 * @see ScanningStrip
 */
class PyScanningStripWrapper
{
public:
  // ***  ATTRIBUTES  *** //
  // ******************** //
  std::shared_ptr<ScanningStrip> ss = nullptr;

  // ***  CONSTRUCTION / DESTRUCTION  *** //
  // ************************************ //
  PyScanningStripWrapper(std::shared_ptr<ScanningStrip> ss)
    : ss(ss)
  {
  }
  virtual ~PyScanningStripWrapper() = default;

  // ***  GETTERs and SETTERs  *** //
  // ***************************** //
  inline std::string getStripId() { return ss->getStripId(); }
  inline void setStripId(std::string const stripId) { ss->setStripId(stripId); }
  /**
   * @brief Like the ScanningStrip::getLeg but obtaining the leg by reference
   * @see ScanningStrip::getLeg
   */
  inline Leg& getLegRef(int const serialId) { return *ss->getLeg(serialId); }
  inline bool isLastLegInStrip() { return ss->isLastLegInStrip(); }
  inline bool has(int const serialId) { return ss->has(serialId); }
  inline bool has(Leg& leg) { return ss->has(leg); }
};

}
