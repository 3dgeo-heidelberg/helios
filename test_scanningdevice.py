
import pyhelios
import tempfile
import os

pyhelios.loggingVerbose()
pyhelios.setDefaultRandomnessGeneratorSeed("42")

# --- Write a minimal inline XML defining a SingleScanner (uses ScanningDevice.cpp) --- #
xml_content = """<?xml version="1.0"?>
<document>
  <scannerSettings id="scanner1" active="true"
                   pulseFreq_hz="100000"
                   scanFreq_hz="50"
                   scanAngle_deg="60"
                   trajectoryTimeInterval_s="0.01"/>
  <survey name="test_scanning_device"
          scene="data/scenes/demo/arbaro_demo.xml#arbaro_demo"
          platform="data/platforms.xml#tripod"
          scanner="data/scanners_tls.xml#riegl_vz400">
    <leg>
      <platformSettings x="0.0" y="0.0" onGround="true"/>
      <scannerSettings template="scanner1"/>
    </leg>
  </survey>
</document>
"""

# --- Save XML to a temporary file --- #
temp_xml = tempfile.NamedTemporaryFile(delete=False, suffix=".xml")
temp_xml.write(xml_content.encode("utf-8"))
temp_xml.close()

# --- Build and run simulation --- #
builder = pyhelios.SimulationBuilder(temp_xml.name, "data", "output")
sim = builder.build()   # returns Simulation directly in this version

print("Running simulation (this should trigger ScanningDevice.cpp)...")
sim.start()             # use .start() instead of .run()
sim.join()              # wait until it finishes
print("Simulation finished.")

# Clean up temporary XML
os.remove(temp_xml.name)
