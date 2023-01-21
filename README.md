# **Line_Follower**

This is the final project for the Introduction to Robotics course taken in the 3rd year at the Faculty of Mathematics and Computer Science, University of Bucharest.

The project involves making a line follower. The project was carried out together with my teammate Bogdan (_A-Bogdan_ on Github). 

At the presentation of the project, our line follower, out of the three attempts, took the **best time of 21.761 seconds**.

## **Contents**
TBA

---

## **Technical Task.**
We were given a robotics kit and a starter code base. We had to assemble the machine and tune the PID.

---

## **Components.**
- 1 x Arduino Uno
- 1 x LiPo battery as a power source 
- 1 x L293D motor driver
- 1 x QTR-8A reflectance sensor
- 1 x Breadboard
- 1 x Chassis
- 1 x Ball caster
- 2 x DC motors
- 2 x Wheels
- wires, zip-ties and screws (according to logic)

---

## **Pictures of the setup.**
![schematic_picture]()
| ![pic1](./pictures/linefollower_pic1.jpeg) | ![pic2](./pictures/linefollower_pic2.jpeg) | 
|:-------------:|:-------------:|
| ![pic3](./pictures/linefollower_pic3.jpeg) | ![pic4](./pictures/linefollower_pic4.jpeg) |

---

## **Picture of the team.**
![pic5](./pictures/linefollower_pic5.jpeg)

---

## **Process explanation.** ([code](./Line%20follower/Line%20follower.ino))
We implemented a machine calibration functionality for each new start. The calibration is based on the information received from the reflectance sensor. The calibration is performed at each new start because the test environment can be different: the colors intensity can be different or the light intensity in the room can be different. So the car adapts to the conditions it is in.

The PID calibrations were made after repeated testing on the test circuits, and the machine wheels speed is adapted accordingly

---

## **Video showing our line follower on a test circuit.** [here](https://youtu.be/AiGLdtWanJ4)

As you can see in the video, our tracker did very well on the winding part of the trail. But, on the straight line section, the car was accelerating like Max Verstappen, but was forgoting to brake, as my colleague Bogdan notes =))

For this we added the following code which causes the car "to brake". 

```
if (error <= -warnningSpotSensorValue) {
    m2Speed -= reduceMotorSpeed;
  } 
  else if (error >= warnningSpotSensorValue) {
    m1Speed -= reduceMotorSpeed;
  }
```

When the sensors in the extremities were registering high error values (i.e. after the straight section there is a tight curve), the car was reduceing its speed considerably.

---

## **Video showing our line follower on the final presentation.** [here](https://youtu.be/Jh95sQ5jw9s)

---