import pyhelios

print(pyhelios.getVersion())
sim = pyhelios.Simulation()
print(
    'Simulation started? {status}'
    .format(status="YES" if sim.isStarted() else "NO")
)
