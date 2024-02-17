
# My System Modeling, Motion Planning, State Estimation, and Control System Design Project
------------------
* **controllers:** 
1) pid controller, 
2) Ackerman's pole placement formula, 
3) Model Reference Adpative Controller (MRAC)
* **data logging:** 
1) data storage in csv file. 
2) Figures plotted using python and saved in png files 
* **numerical solvers:** 
1) Newton-Raphson root finding, 
2) RK4 ode solver
* **serial chain robots:** 
1) Forward kinematics,
2) Inverse kinematics, 
3) Forwards and inverse dynamics, 
4) Joint Trajectory generation, 
5) PID control
* **state estimators:** 
1) Kalman filter, 
2) Extended Kalman filter, 
3) Unscented Kalman filter
* **system models:** 
1) DC motor velocity model, 
2) LTI system model, 
3) Mass-spring-damper system, 
4) Van der Pol oscillator, 
5) wing rock model


## Project Folder Tree (UNDER PROGRSS)

* **bin:** The output executables go here, both for app and for any tests and spikes.
* **build:** This folder contains all object files, and is removed on a `clean`
* **docs:** Any notes, documentation lins
* **include:** All project header files. All necessary third-party header files that do not exists under `/usr/local/include` are also placed here.
* **examples:** Examples demonstrating usage of different source files
* **lib:** Any libs that get compiled by project, third party or any need in development.
* **src:** Application's source files
* **test:** All test code files that reflect source folders