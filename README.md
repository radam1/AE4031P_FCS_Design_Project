# AE4031P_FCS_Design_Project

## Instructions for Running MATLAB and Simulink Files in AFC Assignment
It is important to note that this code builds from code provided for the F16 simulator which was created by RS Russel at the University of Minesotta(see citation in report)

### 1)	Running the Matlab scripts to generate figures and outputs
For running the MATLAB code itself, the main script is HA_ES_MainCode.m. The code is sectioned such that running consecutive sections will eventually produce all of the figures included in the report. For trimming, we selected “y” 3 times for all of the lo-fi models.

### 2)	Running the Simulink Files
The Simulink files need particular workspaces to run, so by running: 

load(“PitchController”)       or      load(“glidescope”)

in the command prompt, the correct workspaces will be loaded. Afterward, the Simulink files can be run via the Simulink run button. 
