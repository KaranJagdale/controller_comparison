## Comparing controllers

Here, I am experimenting with an **inverted pendulum** and a **double mass spring damper** and comparing the performance of varioud controllers on these systems.
The models are stored in [model.py](https://github.com/KaranJagdale/controller_comparison/blob/master/model.py). 

**Inverted pendulum** - We analyze this model in the file [main.ipynb](https://github.com/KaranJagdale/controller_comparison/blob/master/main.ipynb). It is modeled as a rod of uniform mass distribution hinged about a point. The equations of motion are given by:
$$\dot{\theta} = \omega $$
$$\dot{\omega} = \frac{3 g}{2 l} sin(\theta) + \frac{3 (\tau + \beta)}{m l^2} - \frac{3 k \omega}{m l^2}$$
where,
* $\theta$: angle made by the pendulum with the verticle downward direction
* $\omega$: angular velocity of the pendulum
* $\tau$: torque acting on the pendulum (controller input)
* $\beta$: disturbance torque
* $m$: mass of the pendulum
* $k$: angular velocity damping constant
* $l$: length of the pendulum

The measurement is assumed to be the angle of pendulum. The controller goal is to control the pendulum at the upright position (unstable equilibrium equilibrium). We first present the results from PID controller and then for state feedback controller assuming full state is available.

**PID controller** - PID successfully completes the objective only with the P gain as we are trying to control the pendulum at its equibrium point (Generaly I gain is required when when we try to control the system at a non-equibrilium point). The error for PID is defined as $error = \pi - \theta$. Following Gif shows the performance of the PID. For this particular aanimation, I have used $K_p = 10$.
![](https://github.com/KaranJagdale/controller_comparison/blob/master/Invpend_PID.gif)

But in real life the sensors used for the measurements are noisy. In this particular case of inverted pendulum, angle sensor is used to obtain the measurement $\theta$. Thus the output we get from the sensor is given by:
$$\theta = \theta_a + \nu$$

With $\nu \sim \mathcal{N}(0, 0.2)$, i.e., distributed as a Normal distribution with zero mean and 0.2 radian (equal to 11.46 degrees) as the standard deviation. 

Also, there can be some torque disturbance to the system for example if he platform at which the pendulum is on is shaking etc. We model this disturbance in torque again using the Normal distribution with mean 0 and standard deviation equal to $2 Nm$.  
Moreover, in real life the actuator (motor in our case) have limitations on their output, here we assume the motor can give maximum $10 N-m$ of torque.
With this process and noise disturbance and actuator saturation, we get following result.

![](https://github.com/KaranJagdale/controller_comparison/blob/master/Invpend_PID_mes.gif)

**State Feedback Controller** - As a start, state feedback controller is implemented assumming the full state is available without any noise. Using scipy package, we placed the poles of closed loop system in the left-half plane and obtained the controller. The original system in non-linear and thus is linearised in every iteration to obtain the state matrices $(A,B)$. Following anination shows the performance of the state feedback controller. 

![](https://github.com/KaranJagdale/controller_comparison/blob/master/Invpend_SF.gif)

Note that, the target state is achieved way faster than the PID and there is no overshoot. However, we are using full state information for the state feedback controller which is not usually available in real-life scenarios. Also, we haven't put any bound on the controller input which is unrealistic, as the actuators with which we will implement the control will have some limitation on there output. As we go along, we will remove these assumptions and design a more realistic controller that can be implemented on a real system. 




