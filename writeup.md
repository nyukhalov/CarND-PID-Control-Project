# PID Controller Project: Writeup

The goal of the project was to build and tune a PID controller for controlling car's steering angle.

This writeup report describes the approach taken to successfully complete the project.

## Project structure

The `src/` directory contains the source code of two applications:

- `pid` - a program which uses PID controllers to control steering angle and throttle of a vehicle in the simulator.
- `twiddle` - a program which contonuously run simulation for choosing the best hyperparameters for the steering PID controller.

The project's source code files are listed below:

- `PID.h` and `PID.cpp` - an implementation of PID controller.
- `mail.cpp` - the entry point of the `pid` program.
- `twiddle.cpp` - the entry point of the `twiddle` program.

## PID Controller

The implemented PID controller is represented by the `PID` class. A user can create an instance of the class by calling its constructor and passing in `p-`, `i-` and `d-` gain parameters.

The initialization of a PID controller may look like this:

```cpp
double K_p = 0.1;    // p-gain
double K_i = 0.0001; // i-gain
double K_d = 0.3;    // d-gain

PID controller(K_p, K_i, K_d);
```

The class has two methods: `void UpdateError(double cte)` for updating controller's state, and `double TotalError()` for getting the total error.

The methods are used as shown below:

```cpp
steer_pid.UpdateError(cte);
double steer_value = -steer_pid.TotalError();
```

## Tuning

The most difficult part of the project was to choose the right hyperparameters for the steering controller.

The tuning can be done manually, but it would require quite a lot of human time. In order to speed up this process I implemented an automated solution based on the twiddle algorithm given in the classroom.

The `twiddle` program is a client for the simulator which uses twiddle to choose the best hyperparameters.

Each run (stage) in the twiddle is `num_iter` long.
I set `num_iter` to `7000` which is enough to pass the while track at speed of 20MPH.

Once a stage is completed the program sends a restart message (`42["reset",{}]"`) to the simulator in order to restart car's position and speed.

The output of the `twiddle` program shown below shows current stage number, best error and the values of `p` and `d` parameters.

```
stage_no=118  , d_sum=0.484806 , best_err=0.0416151, MSE=0.0832302, p={1.38161, 0    , 0.848355}, d={0.181615, 0.121577, 0.181615}
stage_no=119  , d_sum=0.484806 , best_err=0.0416151, MSE=0.0498773, p={1.01839, 0    , 0.848355}, d={0.181615, 0.121577, 0.181615}
stage_no=120  , d_sum=0.466644 , best_err=0.0416151, MSE=0.0832302, p={1.2  , 0.121577, 0.848355}, d={0.163453, 0.121577, 0.181615}
stage_no=121  , d_sum=0.466644 , best_err=0.0416151, MSE=0.0832302, p={1.2  , -0.121577, 0.848355}, d={0.163453, 0.121577, 0.181615}
```

## Speed controller

I used the PID controller described above to control car's speed. The gain parameters were choosen manually after several experiments:

- `K_p = 0.3`
- `K_i = 0`
- `K_d = 0`

## Conclusion

In this project I implemented and tuned PID controller which successfully drove a car thoughout a virtual track in the simulator.

The `twiddle` algorithm along with the ability of the simulator to programmatically restart car's position allowed me to implement an automated solution for choosing controller's hyperparameters.