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

**PID controller** - PID successfully completes the objective only with the P gain as we are trying to control the pendulum at its equibrium point (Generaly I gain is required when when we try to control the system at a non-equibrilium point). The error for PID is defined as $error = \pi - \theta$. Following Gif shows the performance of the PID.
![](https://github.com/KaranJagdale/controller_comparison/blob/master/Invpend_PID.gif)



