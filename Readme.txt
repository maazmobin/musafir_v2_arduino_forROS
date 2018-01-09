Documents Regarding the Musafir_V2

Arduino:
    • Has equipped with Arduino Mega with a shield design to connect our modules with it.
	https://github.com/maazmobin/Motor-Controller-Arduino-Mega 
	Schematic and layout could be seen from Eagle. So you need an eagle to view it.
    • Arduino code has implemented a ROS and some relevant libraries. The Code could be downloaded from https://github.com/maazmobin/musafir_v2_arduino_forROS 
    • “Libraries” is a folder provided with the code. All libraries are there for the ease of user. 
          How to add library? https://www.arduino.cc/en/hacking/libraries 
        ? ros_lib	  Equipped with the Arduino modules of ROS.
        ? MusafirMotor  Containing the Motor Modules
        ? Encoder use to read encoders. Its a PJRC library https://www.pjrc.com/teensy/td_libs_Encoder.html 
        ? br3ttb-Arduino-PID-Library-fb095d8 Use for PID (Controlling the motors speed)

ROBOT Parameters
Parameters of musafir_v2 is mentioned it Arduino Code (Mentioned above) from line numbers 37 to 42. of the code. Under The heading  “//ROBOT PARAMETERS”. By changing these parameters you can adopt this code for Any other Differential Drive.

ROS (Robot operating System)
    • Topics published by Musafir could be seen from Arduino code line: 19 to 34.
    • Topics Subscribed  by Musafir is only its velocities I.e VL and VR
    • From Ubuntu these topics could be seen by typing rostopic echo on a command prompt.
    • https://github.com/maazmobin/musafir_v2_arduino_forROS  here you will also find a “ROS code” folder where the launch file is. Paste it on on “catkin_ws/src” and do the “catkin_make”. Now by writing on command prompt “roslaunch musafir_v2 musafir_v2.launch” you have launched a “rosserial_python” and a "twist2DiffVel" now its ready to subscribe “cmd_vel”. Launching the teleop will let you drive a Robot. Enjoy!
      
