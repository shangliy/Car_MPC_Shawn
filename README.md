# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
Author: Shanglin
![img](https://github.com/shangliy/Car_MPC_Shawn/blob/master/sample/Screenshot%20from%20MPC_13.mp4.png?raw=true)  
[Video demo](https://youtu.be/i5VVcPQBY5g)
---

## Rubric
###Compilation
Code compile without errors with cmake and make.
###Implementation
####The Model
I implement the MPC model in the MPC.cpp. 
The state includes the location **x**, location **y**, angle **psi**, speed **v**, Cross Track Error **cte** and Orientation Error **epsi**. The dimension of state is **6**.
The actuators include the speed acc **a** and sterring angle **delta**. The dimension is **2**.
The update euqation is shown below:

Code implemention in MPC.cpp:
>fg[2 + x_start + i]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);  
>fg[2 + y_start + i]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);  
>fg[2 + psi_start + i]  = psi1 - (psi0 + v0 / Lf * delta0 * dt);  
>fg[2 + v_start + i]    = v1 - (v0 + a0 * dt);  
>fg[2 + cte_start + i]  = cte1 - ((f_x0 - y0) + v0 * CppAD::sin(epsi0) * dt);  
>fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);  

>**Considering I am using the polfit with degree = 3**  
   >> AD<double> f_x0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);  
   >> AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0,2));  
   
####Timestep Length and Frequency
##### Basic rules:
 The multiple of N (timestep length) and dt (timestep frequency) values decide the length of prediction. 
The larger of N*dt, the longer distance within the prediction.  On the one hand, it provide the car more info to control. On the other hand, the complexity of prediciton increase which make fit unreliable. Besides, the computation cost also increases.
The smaller of N*dt, the shorter  distance within the prediction. If the distance is too small, then the info is not enough to make good control. The car can not follow the path.

Thus, we need to choose the multiple based on the prediction line (Green line in images) with reasonable distance in front of the car. I find the N*dt = 1.3 is reasonbale.

**N = 20 dt =0.2 , distance (Green line) far enough, but high computation.**
![img](https://github.com/shangliy/Car_MPC_Shawn/blob/master/sample/Screenshot%20from%20MPC_N20T02.mp4%20-%201.png?raw=true)

**N = 20 dt =0.2 , sometimes the long distance cause unreliable situation.**
![img](https://github.com/shangliy/Car_MPC_Shawn/blob/master/sample/Screenshot%20from%20MPC_N20T02.mp4%20-%203.png?raw=true)

**N = 10 dt =0.13 , distance (Green line)  is reasonable.**
![img](https://github.com/shangliy/Car_MPC_Shawn/blob/master/sample/Screenshot%20from%20MPC_13.mp4.png?raw=true)

**N = 5 dt =0.1 , distance (Green line)  is too short, the car can not find the right path.**
![img](https://github.com/shangliy/Car_MPC_Shawn/blob/master/sample/Screenshot%20from%20MPC_N5T01.mp4.png?raw=true)

##### dt decision:
Once, we decide the multiplication, then I test the dt. If the dt is small, the reaction time between two temporal stage is small which make the car react fast to the path change. But if the dt is too small, the care become unstable. From experience, when dt = 0.05, the car is unstable, if the dt =0.5, the car can not fit the path at time. After test, I choose dt = 0.13. Thus, N =10.

####Polynomial Fitting and MPC Preprocessing

#####  Polynomial Fitting:
I use Polynomial fitting with degree = 3
> auto coeffs = polyfit(ptsx_vector, ptsy_vector, 3); 

##### MPC Preprocessing:
Need to transform the world coordinates to Car coordinates
>        for (int i = 0; i < ptsx.size(); i++) {
>
>            double dtx = ptsx[i] - px;
>            double dty = ptsy[i] - py;
> 
>            ptsx_vector[i] = dtx * cos(psi) + dty * sin(psi);
>            ptsy_vector[i] = dty * cos(psi) - dtx * sin(psi);
>
>          }

#### Model Predictive Control with Latency
1. To handle the latency, I use the model predict model. The model can react to the change immediately.
2. To increase the preformance of the model, I add high weights to the cost of cte and epsi. Then model will perform fast to minimize the difference.
> 
>//value ranges for penalties:
>    for (int i = 0; i < N; i++) {  
>       fg[0] += 1000 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);  
>        fg[0] += 1000 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);  
>       fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);  
>    }  
>    // penalize the use of actuators  
>    for (int i = 0; i < N-1; i++) {  
>        fg[0] += 10 * CppAD::pow(vars[delta_start + i], 2);  
>        fg[0] += 10 * CppAD::pow(vars[a_start + i], 2);  
>    }  
>    // penalize big value gaps in sequential actuations  
>    for (int i = 0; i < N-2; i++) {  
>          fg[0] += 100 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);  
>          fg[0] += 100 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);  
>    }  

3. Algorithm to solve the latency:
The implemented algorithm is to project the vehicle state 100ms into the future before calling the MPC.solve() function. Since the state variables are computed in vehicle coordinate system, the following projections could be made on the state variables. Record the **steer_value** and **throttle_value** using mpc.steer_value_pre and mpc.throttle_value_pre, then calculate the "predicted" value considering the latency.

> dt = 0.1 // equivalent to 100ms  
> Future_x = v * dt;  
> Future_y = 0;  
> Future_psi = -  mpc.steer_value_pre / Lf * dt;  
> Future_v = v + mpc.throttle_value_pre * dt;  

After that, I put the predicted future value in MPC.solve() function.

> state_cur << Future_x, Future_y, Future_psi, Future_v, cte, epsi;  
> auto control = mpc.Solve(state_cur, coeffs);  


4. The car work well with 100ms latency.  
 
 
#### Simulation
No tire may leave the drivable portion of the track surface.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


