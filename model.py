import numpy as np
import scipy as sc
import matplotlib.pyplot as plt

class massSpringDamper():
  def __init__(self, m, c, k):
    self.m, self.c, self.k = m,c,k

  def dyn(self, y, t, u):
    x,v = y
    dydt = [v, u - (self.c/self.m)*v - (self.k/self.m)*x]
    return dydt

  def input(self, y, t):
    #define necessary constants for the inout here
    f = 1
    w = 6.28
    u = np.cos(w*t)
    return u

  #simulate method can be used if the input can be pre-defined (non-state feedback based) 
  def simulate(self, tI, tF, yI): 
    tSamp = 0.05 #sampling time 
    yC = yI
    xA = [yC[0]]
    vA = [yC[1]]
    tGrid = np.arange(tI, tF, tSamp)
    for t in tGrid[1:len(tGrid)]:
      #print(t)
      u = self.input(yC, t)
      yN = sc.integrate.odeint(self.dyn, yC, [t-tSamp,t], args=(u,))
      #print('yN',yN)
      xA.append(yN[yN.shape[0]-1,0])
      vA.append(yN[yN.shape[0]-1,1])
      yC = yN[yN.shape[0]-1,:]

    plt.figure(1)
    plt.subplot(211)
    plt.plot(tGrid, xA)
    plt.subplot(212)
    plt.plot(tGrid, vA)
    print(xA[0:10])
    
