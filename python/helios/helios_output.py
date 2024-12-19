# import _helios 

# class HeliosOutput:
#     def __init__(self, measurements, trajectories, outpath, outpaths, finished):
      
#         if isinstance(measurements, list):
#             self.measurements = _helios.MeasurementVector(measurements)
#         elif isinstance(measurements, _helios.MeasurementVector):
#             self.measurements = measurements
#         else:
#             raise TypeError("Expected list or MeasurementVector for measurements")

#         if isinstance(trajectories, list):
#             self.trajectories = _helios.TrajectoryVector(trajectories)
#         elif isinstance(trajectories, _helios.TrajectoryVector):
#             self.trajectories = trajectories
#         else:
#             raise TypeError("Expected list or TrajectoryVector for trajectories")

#         self.outpath = outpath
#         self.outpaths = outpaths
#         self.finished = finished

#     def __del__(self):
#         # Perform any necessary cleanup
#         pass
