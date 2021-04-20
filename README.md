# Lab-ArmSupport
This code scales the 1DoF Admittance control code to 3DoF to control a motorized arm support being devloped in the lab.
The intension of the arm support is for home based stroke therapy.

Code comprised of 4 classes:
* Admittance Model
  - Force acting on mass at the end effector.
* Force Sensor
  - 1kHz tri-axis force sensor at the end effector.
* Robot Control
  - iKine, fKine, and motor communication.
* Serial Communication
  - Outgoing packet protocol for data logging.
  - Incoming packet protocol for mass/damping modification and outgoing packet configuration.

Code by Erick Nunez
Supervised by Sergei Adamovich, PhD
