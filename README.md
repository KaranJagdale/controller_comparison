##Comparing controllers

Here, I am experimenting with an **inverted pendulum** and a **double mass spring damper** and comparing the performance of varioud controllers on these systems.
The models are stored in [model.py](https://github.com/KaranJagdale/controller_comparison/blob/master/model.py). 

**Inverted pendulum** - We analyze this model in the file [main.ipynb](https://github.com/KaranJagdale/controller_comparison/blob/master/main.ipynb). It is modeled as a rod of uniform mass distribution hinged about a point. The equations of motion are given by:
$$ \dot{\theta} = \omega $$
$$ \dot{\theta} = \frac{3 g}{2 l} sin(\theta) + \frac{3 (\tau + \beta)}{m l^2} - \frac{3 k \omega}{m l^2}$$