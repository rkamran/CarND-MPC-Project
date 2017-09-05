## Project - Model Predictive Control
 
#### Rubric 1 - Compilation
 Project can be compiled using using ``cmake`` and ```make``` without any issues.
 
#### Rubric 2 - Implementation

***A. The model:***
 
The model I used here is what we studied in the class and implemented as quiz. Given the previous state at time *t* the
model calculates the state at time *t+1*

Here are the equations for the MPC model.


    xt+1 = xt + vt * cos(ψt) * dt
    yt+1 = yt + vt * sin(ψt) * dt
    
    ψt+1 = ψt + vt/Lf * δt * dt    
    vt+1 = vt + at * dt
    
    ctet+1 = f(xt) - yt + (vt * sin(ψt) * dt)
    eψt+1 = ψt - ψdest + ( vt/Lf * δt * dt)
    

where **x,y** are the position of vehicle and t is the time.  **ψ** denotes the heading direction and **v** is the velocity
 of the car. **Lf** is the distance between vehicle center of mass to the front wheel. **cte** and **eψ** are the cross 
 track error and error in orientation.
 
 
***B. Time step Length and Elapsed Duration:***

```
size_t N = 20;
double dt = 0.2;
```    

Using N and dt we define our prediction horizon. The goal is to have a smooth drive while not introducing unmanageable latency.
  
Very short horizon is responsive but can suffer from accuracy and very long horizon could introduce a lot of latency in the system. 
The value worked great for me is N=20 with dt = 0.2. I have also tried N = 30 so span the horizon even further but it didn'
 work an car start swinging harder. 
 
 

***C. Polynomial Fitting and MPC Preprocessing:***

Since starting at origin with zero orientation angle  a  polynomial is fit to keep it simple
  
```
  Eigen::VectorXd ptsxASvector = Eigen::VectorXd::Map(ptsx.data(), 6);
  Eigen::VectorXd ptsyASvector = Eigen::VectorXd::Map(ptsy.data(), 6);

  //--> Vehicle coordinate
  for (int i = 0; i < ptsxASvector.size(); i++) {
    double x = ptsxASvector[i]-px;
    double y = ptsyASvector[i]-py;
    ptsxASvector[i] = x * cos(0-psi) - y * sin(0-psi);
    ptsyASvector[i] = x * sin(0-psi) + y * cos(0-psi);
  }

  //--> Fitting a polynomial -- Using 3rd degree
  auto coeffs = polyfit(ptsxASvector, ptsyASvector, 3);
  
  //--> cte and epsi
  double cte = polyeval(coeffs, 0);
  double epsi = -atan(coeffs[1]);

  //--> Zero state -- current state
  Eigen::VectorXd state(6);
  state << 0, 0, 0, v, cte, epsi;

  //--> Solve using MPC
  auto x1 = mpc.Solve(state, coeffs);
  
```
  
***D. Model Predictive Control with Latency:***


First the choice of dt 0.2 over 20 to keep the horizon optimal. Speed limit of 70m/h and added penalty to slow down on 
tight corners to compensate the lag of 0.1sec.

```
for (int index = 0; index < N; index++) {
    fg[0] += 2500 * CppAD::pow(vars[cte_start + index] , 2);
    fg[0] += 2500 * CppAD::pow(vars[epsi_start + index] , 2);
    fg[0] += CppAD::pow(vars[v_start + index] - ref_v, 2);
}

for (int index = 0; index < N - 1; index++) {
    fg[0] += 100 * CppAD::pow(vars[delta_start + index], 2);
    fg[0] += 10 * CppAD::pow(vars[a_start + index], 2);
}

for (int index = 0; index < N - 2; index++) {
    fg[0] += 100 * CppAD::pow(vars[delta_start + index + 1] - vars[delta_start + index], 2);
    fg[0] += 10 * CppAD::pow(vars[a_start + index + 1] - vars[a_start + index], 2);
}
```


To account for latency we use a previous actuation to control the car. The following line for code were added to acount for latency

```
  //-->Adjust or latency
  if (index > 1) {
	  delta0 = vars[delta_start + index - 2];
	  a0 = vars[a_start + index - 2];
  }

```

#### Rubric 3: Simulation ####

A full round across the track can be watched in the video here.

[MPC Simulation - YouTube](https://youtu.be/mnH7fkl_yyk)