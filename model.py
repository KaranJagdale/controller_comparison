import numpy as np
import scipy as sc
import matplotlib.pyplot as plt
import random
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
      
      u = self.input(yC, t)
      yN = sc.integrate.odeint(self.dyn, yC, [t-tSamp,t], args=(u,))
      
      xA.append(yN[yN.shape[0]-1,0])
      vA.append(yN[yN.shape[0]-1,1])
      yC = yN[yN.shape[0]-1,:]

    plt.figure(1)
    plt.subplot(211)
    plt.plot(tGrid, xA)
    plt.subplot(212)
    plt.plot(tGrid, vA)
    print(xA[0:10])

class InvertedPendulum:
    def __init__(self, m, l, g, k, isDisturbance, procNoiseCov, mesNoiseCov) -> None:
        self.m, self.l, self.g, self.k, self.isDisturbance = m, l, g, k, isDisturbance
        self.CMatt = np.array([[1,0]])
        self.DMatt = np.array([[0]])
        self.procNoiseCov = procNoiseCov
        self.mesNoiseCov = mesNoiseCov

    def thetaDDot(self, Theta, Tau):
        return 1.5*self.g/self.l*np.sin(Theta) - Tau*3/self.m/self.l**2
    
    def DynSS(self, y, t,Tau,disturb, isPrint = False):
        Theta, Omega = y
        # disturb = np.random.normal(0, self.procNoiseCov, 1)[0]
        # disturb = disturb*(self.isDisturbance * isDisturb)
        #print(disturb)
        dydt = [Omega, -1.5*self.g/self.l*np.sin(Theta) + 3*(Tau + disturb)/self.m/self.l**2 - 3*self.k*Omega/self.m/self.l**2]
        if isPrint:
           print(dydt)
           print('fterm - ', -1.5*self.g/self.l*np.sin(Theta))
           print('sterm - ', 3*(Tau + disturb)/self.m/self.l**2)
           print('tterm - ', 3*self.k*Omega/self.m/self.l**2)
        return dydt
    
    def AMatt(self, Theta):
       mat = np.array([[0, 1], [-3/2*self.g/self.l*np.cos(Theta), -3*self.k/self.m/self.l**2]])
       return mat
    
    def BMatt(self):
       mat = np.array([[0], [3/self.m/self.l**2]])
       return mat
    
    def measurement(self,state):
       return np.dot(self.CMatt, state) + np.random.normal(0, self.mesNoiseCov, 1)[0]

    def nextState(self, y,Tau, Ts, disturb = 0):
        y0 = y
        sol = sc.integrate.odeint(self.DynSS, y0, [0, Ts], args=(Tau, disturb))
        return sol[1,:]
    
    def getDiscreteDynMatrix(self, Theta, sim_dt):
       d_syst = sc.signal.cont2discrete((self.AMatt(Theta), self.BMatt(), self.CMatt, self.DMatt), sim_dt)
       Phi, Gamma, C_d = d_syst[0], d_syst[1], d_syst[2]
       return Phi, Gamma, C_d


class DoubleMassSpringDamper():
    def __init__(self, m1, m2, b, c1, c2, k, isDisturbance = False) -> None:
        self.m1, self.m2, self.b, self.c1, self.c2, self.k, self.isDisturbance =  m1, m2, b, c1, c2, k, isDisturbance

        self.AMat = np.array([[0, 0, 1, 0],
                     [0, 0 ,0, 1],
                     [-k/m1, k/m1, -(b+c1)/m1, b/m1],
                     [k/m2, -k/m2, b/m2, -(b+c2)/m2]]).reshape(4,4)
        
        self.BMat = np.array([0, 0, 1/m1, 0]).reshape(4,1)

        self.CMat = np.array([-1, 1, 0, 0])

        self.mu = 0.1

        self.g = 9.86  

    def DynSS(self, y, t, F):
        x1, x2, x1Dot, x2Dot = y

        dydt = np.dot(self.AMat, np.array(y).reshape(4,1)) + np.dot(self.BMat, F) + self.disturbance(y, t)
        #print('from DynSS', 'a', np.dot(self.AMat, np.array(y).reshape(4,1)), 'b', np.dot(self.BMat, F), 'c', self.disturbance(y, t))
        # print('from DynSS', 'BMat - ', self.BMat, 'F', F, 'b', np.dot(self.BMat, F))
        # print('dydt',dydt)
        # print('DynSS over')
        dydt = dydt.reshape(1,4).tolist()
        return dydt[0]

    def disturbance(self, y, t):
        if self.isDisturbance:         
          x1, x2, x1Dot, x2Dot = y        
          disturb = [0, 0, -np.sign(x1Dot)*self.mu*self.g, -np.sign(x2Dot)*self.mu*self.g]*self.isDisturbance
          disturb = np.array(disturb).reshape(4,1)
          return disturb
        else:
           return 0

    def nextState(self, y, F, Ts):
        y0 = y
        sol = sc.integrate.odeint(self.DynSS, y0, [0, Ts], args=(F,))
        return sol[1,:]    

       
       

