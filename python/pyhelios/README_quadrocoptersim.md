## Quadcopter flight simulations

This is a experimental feature to simulate Quadcopter flights interactavely. 
In the beginning, several points in the scene are selected in a photo of the scene by double click. The copter will always fly in a specified height above the ground.
Using a slightly adapted version of the Quadcopter Simulation package https://github.com/bobzwik/Quadcopter_SimCon , the trajectory is simulated and saved in format x-y-z-t-roll-pitch-yaw. 
After this, the live simulation starts (helios-live).
Run the script (from the helios directory) with 

python ./python/pyhelios/live_quadro.py data/surveys/demo/box_survey_interp.xml


required additional python packages are open3d, opencv and pydy


 
