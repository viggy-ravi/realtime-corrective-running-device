# biodesign-running-form-device
 Seven Sensor Biodesign Project
 Sarah Bi, Jessica Fritz, Vignesh Ravindranath, Aradhana Sridaran, Vikas Yerneni

 Click [here](https://thedailytexan.com/2019/10/28/biomedical-engineering-students-design-device-to-correct-running-form) to see our project featured in the Daily Texas (campus newspaper)!!  
 
 Device takes in four pressure sensor inputs (two on the balls of the feet and two 
 on the heels) to determine if a heel-striking occurs at midstance. Once pressure
 has been detected, two inertial sensor inputs are read (one from the hip and one 
 from the ankle) and stored, and calculations for the hip-ankle angle at midstance
 are made.
  
 Sound outputs occur for heel striking (heel pressure sensor goes high before pressure
 sensor on balls of feet) and hip-ankle angle greater than 15 degrees (0.261799 rads).
 
 Libraries Used:
 - Bolder Flight System MPU9250: https://github.com/bolderflight/MPU9250
 - MatrixMath: https://playground.arduino.cc/Code/MatrixMath/
 
 Devices Used:
 - MPU9250 9-Axis 9-DOF 16 Bit Gyroscope Acceleration Magnetic Sensor
 - Force Sensitive Resistor DF9-40 High Accuracy Resistance (Pressure Sensor)
