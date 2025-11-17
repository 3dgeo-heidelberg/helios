from pyhelios import SimulationBuilder

print("Building simulation...")

builder = SimulationBuilder("data/surveys/demo/tls_arbaro_demo.xml", "data", "output")
builder.loadXml("data/surveys/demo/tls_arbaro_demo.xml")
sim = builder.getSimulation()

print("Running simulation...")
sim.run()

print("Done.")
