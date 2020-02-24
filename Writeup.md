# **Extended Kalman Filter Project**

In this project EKF is implemented in C++ to estimate the position of an object using the simulated LASER and RADAR measurements.

[image1]: ./output/dataset1.png "Dataset 1"
[image2]: ./output/dataset1_laser.png "Dataset 1 LASER"
[image3]: ./output/dataset1_radar.png "Dataset 1 RADAR"
[image4]: ./output/dataset2.png "Dataset 2"

### **Initializing the Matrices**  

 Jacobian RADAR - Hj

       [ 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1 ]

Measurement covariance matrix LASER - R_laser_
   
       [ 0.0225, 0,
         0, 0.0225 ]

Measurement covariance matrix RADAR - R_radar_
  
       [ 0.09, 0, 0,
         0, 0.0009, 0,
         0, 0, 0.09 ]

Observation Matrix LASER - H_laser_ 

       [ 1, 0, 0, 0, 
         0, 1, 0, 0 ]

State Covariance Matrix - P = Identity(4, 4)  
State Transition Matrix - F = Identity(4, 4)  
Process Covariance Matrix - Q = Identity(4, 4)  


### **Root Mean Square Error (RMSE)**
---
The RMSE of Dataset1 and Dataset2 are as follows  
Calculated RMSE is less than the required RMSE in the project rubric. 

|State|  Dataset1 |Dataset2|
|:----|:---------:|:------:|
| x   | 0.0964    | 0.0726 |
| y   | 0.0853    | 0.0965 |
| v_x | 0.4154    | 0.4216 |
| v_y | 0.4316    | 0.4932 |


For the Dataset 1 EKF Filter is tested for the scenarios where only LASER measurements or RADAR measurements are available for the UPdate step. The results are tabulated below. Clearly RMSE is better when both sensor measurements are available and further, position estimation with just RADAR measurements are worse compared to lone LASER measurements.

|State|  Both sensors | LASER  |  RADAR  |
|:----|:-------------:|:------:|:-------:|
| x   | 0.0964        | 0.1501 | 11.5189 |
| y   | 0.0853        | 0.1151 | 7.8312  |
| v_x | 0.4154        | 0.6954 | 9.9059  |
| v_y | 0.4316        | 0.5379 | 8.5455  |



### **Video Links**

Recorded position estimates for the given simulated data are as follows.
| Dataset 1| Only LASER| Only RADAR|
|:-:|:-:|:-:|:-:|
|[![alt text][image1]](<https://youtu.be/QUMj1lt3QBk>)|[![alt text][image2]](<https://youtu.be/Pd7SEuNDVD0>)|[![alt text][image3]](<https://youtu.be/M-4JYOh9oMI>)|

Dataset 2
[![alt text][image4]](<https://youtu.be/0q4buS44Uno>)
