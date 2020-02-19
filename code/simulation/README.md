
We ran our EKF algorithm in python on Google Collab.  We decided to use Google Collab to make code sharing much easier. 

The python code takes in the sensor outputs of the robot in a text file format. These sensor outputs are gathered manually from the robot due to issue interfacing over wifi and trying to run the code locally on the robot.

The text file format is:
front sensor, right sensor, gyroscope z angular velocity, angle of rotation, PWM left servo, PWM right servo

The code then creates the Kalman gains for each of the measurement outputs and estimates the states. 
