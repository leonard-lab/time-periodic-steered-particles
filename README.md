time-periodic-steered-particles
===============================
Oscillating fish algortithm code for DCSL at Princeton University

Description and Resources:
==============================
The files in this repository provide control laws and code for the Miabot robots to follow the oscillating-speed behavior of fish schools. This dynamics, behavior, and control laws are described in "Alternating Spatial Patterns for Coordinated Group Motion" by Daniel Swain, Naomi Leonard, Iain Couzin, Albert Kao, and Rodolphe Sepulchre, 2007. More details on the control laws and dynamics related to this oscillatory behavior can also be found in the 2009 paper by Swain and Leonard, "On the Trajectories and Coordination of Steered Particles with Time-Periodic Speed Profiles." Additional general information concerning coordinated group motion of steered particles can be found in "Stabilization of Planar Collective Motion: All-to-All Communication" by Sepulchre, Leonard, and Derek Paley. Swain's PhD thesis also provides extensive explanation. 

Code Information:
=============================
The code closely follows the control laws outlined in the listed papers. MiabotFish.m provides a run script to set up the oscillating fish school object and actuate Miabot robot control. The oscillatingFish.m class provides the control law for the Miabots, as well as a couple other helpful simulation methods. 

The control law properties such as the different control constants, fish natural frequencies, and fish speed oscillation amplitude are all public and can be manipulated. Additionally, the oscillatingFish.m class provides several different options for what type of behavior is desired - such as splayed, synchronized, or arbitrary heading or speed phase angles. 

The simulate method in oscillatingFish.m simulates the steered particle motion for the given run time and graphs the resultant expected trajectory. It can also graph other characteristics such as the speed phase history, heading angle history, and the particles position on the ellipse speed phase locus.  

Dependencies:
==============================
MiabotFish.m requires Miabots.m and its dependences. Those files can be downloaded at https://github.com/leonard-lab/dcsl-matlab-api.git. oscillatingFish.m has no outer dependencies. 

Running:
=============================
To run code on Miabots: 

1. Launch the ROS GUI to connect the computer to the Miabot robots and camera tracker. Terminal Commands: %roslaunch dcsl_miabot_main miabot_main.launch (miabot_main3.launch for 3 robots) 

2. Turn on each robot (one by one), and connect them through bluetooth using the connect options in the ROS GUI. The screen will freeze temporarily before each connection.

3. Place robots in field, and check that each one is picked up by the camera and properly labeled.

4. Open MiabotFish.m and run the first cell to initialize an oscillatingFish object. Next run the cell containing the call to Miabots.m to start commands to the Miabot robots. Make sure the the 'sim' option in the Miabots call is set to false. 

5. When the robots have finished, the trajectory can be plotted by running the plot histories cell in the script.  
