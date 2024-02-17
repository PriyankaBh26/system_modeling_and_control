
# My System Modeling, Motion Planning, State Estimation, and Control System Design Project
------------------
* **controllers:** pid controller, Ackerman's pole placement formula, Model Reference Adpative Controller (MRAC)
* **data logging:** data storage in csv file. Figures plotted using python and saved in png files 
* **numerical solvers:** Newton-Raphson root finding, RK4 ode solver
* **serial chain robots:** forward and inverse kinematics, dynamics, trajectory generation, PID control
* **state estimators:** Kalman filter, Extended Kalman filter, Unscented Kalman filter
* **system models:** DC motor velocity model, LTI system model, Mass-spring-damper system, Van der Pol oscillator, wing rock model


## Project Folder Tree (UNDER PROGRSS)

* **bin:** The output executables go here, both for app and for any tests and spikes.
* **build:** This folder contains all object files, and is removed on a `clean`
* **docs:** Any notes, documentation lins
* **include:** All project header files. All necessary third-party header files that do not exists under `/usr/local/include` are also placed here.
* **examples:** Examples demonstrating usage of different source files
* **lib:** Any libs that get compiled by project, third party or any need in development.
* **src:** Application's source files
* **test:** All test code files that reflect source folders