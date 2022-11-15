# IM Force Sensor Module

This repository maintains the basic usage of OnRobot-Hex-qc force torque sensor, and external force detection algorithms in IM project. 



## Build

1. dependency  
Eigen and pthread needed.

2. build  
Just colcon build it.


## Test
   ```
   cd ~/RootPath
   ./build/ft_sensor/testFtSensor   
   ```
Basically it will test the data acquisition of force-torque sensor, and h_factor computation in AdmittanceController.  

One can use the raw data from sensor to directly perform the external force detection, or use the h_factor to do so, the later one requires persistent contact, thus sharp shock will be ignored. 

> h_factor(humanInteractionRatio) represents the energy tank of the perturbance(external force caused by collision or human interaction). When persistent contact exists, h_factor will keep increasing till hits 1,which indicate the robot should stop the current task since a strong collision or human interaction was detected. Once the external force disappeared, the h_factor will decrease to 0 with a dissipation rate, and then robot can continue performing the previous task since it's regarded totally safe.   


## Params tuning

1. Force sensor related params
   ```
   // 1. sampling rate of the sensor
   // default to 100 Hz
   setSamplingRate(int frequency); 
   
   // 2. LPF cutoff frequency
   // default to 15 Hz
   /* 0 = No filter; 
      1 = 500 Hz; 2 = 150 Hz; 
      3 = 50  Hz; 4 = 15  Hz; 
      5 = 5   Hz; 6 = 1.5 Hz */
   setFilterType(int frequency);

   // 3. Biasing on/off (take current data as baseline or not)
   // default to false
   enableBiasing(bool biasing_on);
   ```

2. AdmittanceController params  
   D_a[6x6 matrix]

   ```
   // damping matrix for admittance control
   // default to diag{20, 20, 20, 10, 10, 10}
   Eigen::MatrixXd D_a; 
   ```

   M_a[6x6 matrix]

   ```
   // mass matrix for admittance control
   // default to diag{15, 15, 15, 7.5, 7.5, 7.5}
   Eigen::MatrixXd M_a; 
   ```

   Pd_tilde[double]
   ```
   // dissipation rate of h_factor
   // default to 0.25
   double Pd_tilde; 
   ```

   dt[double]
   ```
   // time interval of the controller, dt = 1.0 / frequency
   // default to 1.0 / 100
   double dt; 
   ```

   E_thres[double]
   ```
   // threshold of the energy tank, defines the dead zone of collision detection
   // default to 10
   double E_thres; 
   ```

   E_max[double]
   ```
   // upper bound of the energy tank
   // default to 50
   double E_max; 
   ```

## examples
examles/onrobottcp.c and examples/onrobotupd.c are the simple demo showing how to communicate to the sensor with an Ethernet UDP/TCP using standard C functions.  

   ```
   gcc onrobotudp.c -o upd_demo
   gcc onrobottcp.c -o tcp_demo
   ```


## scripts
Contains python version of all the stuff discussed above, along with a qt plotter for realtime debugging. 

## TODO
Integrate h_factor control into the planner, to perform a compliant behavoir with human interaction as a passive follower, and execute the predefined trajectory-tracking task when no perturbance detected, which is actually a stiff impedance controller.  
