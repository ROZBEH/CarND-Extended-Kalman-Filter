# Extended Kalman Filter Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
In this project we are going to use Extended Kalman Filter for tracking objects around self driving car. Laser and Radar data will be used as input to extended Kalman Filter. 
Pipeline
---

*The overall pipeline along with the results will be described here!*

<br>

I. First sensor data will be read. Depending on the sensor type, we apply conversion to convert polar coordinate data into cartesian. For your note, Radar data is in polar coordinates and Laser data is in cartesian coordinates.

II. Firt loocation (Px,Py) and velocity (Vx, Vy) of object will be predicted.

III. Once the location is predicted, sensor input data will be used to update the location of the object. After each update step, update values will be pushed into an estimate array in order to calculate the root mean square error(rmse). As can be seen in the following youtube video the errors for (Px, Py, Vx, Vy) are around [] at the end of the simulation.

IV. I also tried cases were just one of the sensors was active and measurements of the other senor was not considered during update and prediction step. We observe higher rmse when one of the sensors is inactive, this is because we have less information about the object, as a result there will be more uncertainty in calculations and the performance will be degraded. Comparing Laser and Radar based on rmse, it appears laser performs better because it has lower rmse error.

<!-- [![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/EAdp8r0g58M/0.jpg)](https://www.youtube.com/watch?v=EAdp8r0g58M) -->
[![Demo CountPages alpha](https://j.gifs.com/ZVZwnv.gif)](https://www.youtube.com/watch?v=EAdp8r0g58M)

<br>The following shows driving next to the edge of the curve and bringing the car back to the center of the road</br>
<p align="center"><img src="examples/curv_1.jpg" width = "350" alt="Combined Image" />
<img src="examples/curv_1_2.jpeg" width = "350" alt="Combined Image" /> </p>

</br>
<br></br>


