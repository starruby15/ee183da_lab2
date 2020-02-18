This is the code that drives the robot. 

This is made with the skeleton provided by Professor Mehta from Lab 1. 

Included Files: 

paperbot.ino - contains main loop for the paperbot, created on Arduino.  It has code to drive robot with various movement 
functions.  It also contains the code for interfacing with the sensors.  It runs both the loop to connect over IP as well as 
loops for testing the sensors and testing various simulation runs. 

MPU9250.h and MPU9250.cpp - contains helper functions to interface with the IMU and use the gyroscope data. 
We were having issues interfacing with the gyroscope so we found code online to help with it.
Full credit is given for MPU9250.h and MPU9250.cpp in the comments at the top of both files to Borderflight. 
