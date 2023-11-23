# softTetrahedralSim_Pybullet
PyBullet simulation setup for a soft tetrahedral robot.

## Requirements
1) PyBullet (https://pybullet.org/wordpress/)
2) Python 3.6+

## Usage
Main file contains sample code for roling the tetrahedra robot. To control the robot, user has to provide
1) Bending angle (str) and Direction (dir)
2) str = [0, 0.39] rad | dir = [0, 3.14] rad

The example bends the each legs in a sequence manner to obtain the rolling motion. This sequence is derived from the Rolling_gait.py file.
