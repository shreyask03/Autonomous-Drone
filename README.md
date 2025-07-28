# Explanations And Context About The Project

# Summary
This repository contains an Arduino based flight controller program for a drone with 4 propellers (quadrotor). 
It utilizes 3 axis PID feedback control architecture that has a cascaded structure. 
The lowest level of PID control takes as input a desired angular rotation rate and compares it to the measured rate 
from the onboard gyroscope to output motor speed corrections that reduce the difference in rates, the error, down to 0. This control loop operates at 250 Hz
for smooth resolution in control effort. The next level of control takes as input a desired angle and compares it to the measured angle 
from a filtered inertial measurement unit (IMU) output that combines both gyroscope and accelerometer data for smooth, noiseless readings.

# CAD Models
Furthermore, the repo also contains SOLIDWORKS parts/assembly of the drone frame, alongside .STEP214 files inside the CAD_Models folder 
that can be used to 3D print the same frame that I used. Keep in mind that this will be updated as I go, 
but I might leave older working frame iterations to make repairs easier or for people to get into the project from the beginning.

# WiFi Communication Setup Explanation
With that being said, after building and assembling the frame along with all electronic components, 
I ran into a frustrating issue that I have since resolved. For more context, it is essential that the drone's PID control system is tuned
properly, prior to being able to hold a successful flight test, because otherwise the drone will be completely uncontrollable and may crash
or worse, injure somebody. So I would suggest watching youtube videos on what each term in PID is doing, and how the tuning process looks to 
get an idea of what is going on before attempting anything yourself.

Basically, the standard way to tune the control system would be to plug the arduino in, change the initialized PID objects values, 
and then test it on a stand, rinsing and repeating until steady control is achieved. 
The problem lies here: There may be lots of fine tuning involved, and having to turn the drone off, connect it to the Arduino IDE, 
change the control gains, upload code, then test, is a pretty long process. Although not a problem when doing it a couple times.
I personally had to go back and forth countless times, and after a certain point I decided to implement a new system that would make things
far easier for me. I developed communication over WiFi using TCP or request based communication between the flight controller and my 
base station (a laptop in my case). I incorporated the communication through a custom graphical user interface (GUI) on the base station
to easily send tuning gains over once connected to the flight controller, and it shows the latest tune after a request is sent automatically
upon both opening the GUI as well as after sending a new tune over. This siginificantly shortened the time needed to tune and 
I was finally able to tune my drone and get things working properly.

Here ends the final stage for manual control using a radio.

# Beginning Drone Autonomy Development via ROS 2 Humble
After this point, I planned on making the drone fly autonomously using smart waypoint navigation. 
I realized what else I could do with the right sensors and ended up finding out about ROS.

From here, I've setup a ROS 2 Humble workspace on Ubuntu 22.04.5 LTS and have created a custom package for use
in simulations, as well as visualizations when I run LiDAR-based SLAM and visual SLAM in the near future from my base station
using publisher nodes set up on an onboard Raspberry Pi, acting as a sensor bridge. This allows me to reduce
computational load onboard the drone, therefore reducing the power consumption of an already power hungry Raspberry Pi
in order to squeeze out as much flight time as possible.

I plan on incorporating a method for simulating a 3D LiDAR scan using a 360 degree 2D LiDAR eventually to visualize
dense 3D reconstructions of the environments the drone flies through. 

# WILL UPDATE FROM HERE
