import numpy as np
import matplotlib.pyplot as plt

class OneDofSystem(object):
    def __init__(self, x, v, biasAcceleration=0):
        self.x = x
        self.v = v
        self.biasAcceleration = biasAcceleration
        
    def step(self, u, h):
        # Returns the updated position x and velocity v after one time step h
        a = self.biasAcceleration + u
        self.x = self.x + h*self.v + 1./2.*a*h**2 # calculate new position
        self.v = self.v + h*a # calculate new velocity
        
    def sense(self):
        # Returns the current position
        return self.x
    
def response(system, k_p, k_d, k_i):
    sys = system(0, 0)
        
    x_target = 1 # Set target position
    delta_x_target = 1 # Updating the target 
    
    h = 0.01 # dt
    x_old = sys.sense() # initial position of the system
    
    e_old = 0 # initial error
    e_i = 0 # initial intergral error
    
    x_list = []
    target_list = []
 
    for c in range(1000):       
        # obtain current position of system         
        x = sys.sense()
        
        # half-way update the target         
        if c==500:
            x_target += delta_x_target
        
        # Implement the discretized PID loop equations 
        ### START CODE HERE ### (~ 4 lines of code)
        e = x_target - x # Calculate the error in position 
        e_d = (e - e_old)/h # Calculate derivative of the error
        e_i = e_i + e*h # Calculate integral of the error  
        u = k_p*e + k_i*e_i + k_d*e_d # Calculate the output of the PID
        ### END CODE HERE ###
        
        # apply control effort, u (acceleration)
        sys.step(u, h)
        
        x_old = x # store previous position 
        e_old = e # store previous error 
        
        # store position, target and time for plotting reasons         
        x_list.append(x)
        target_list.append(x_target)
        time = np.arange(1000)*h
    return time, np.array(x_list), np.array(target_list)

# Tune the PID equations
### START CODE HERE ### (3 lines of code)    
k_p = 5
k_d = 30
k_i = 0
### END CODE HERE ###

systemFun = lambda x, v : OneDofSystem(x,v, 0)
time, position, target = response(systemFun, k_p, k_d, k_i)
    
plt.plot(time, position)
plt.plot(time,np.ones(time.shape)*target)
plt.xlabel("time (in s)")
plt.ylabel("position (in m)")

plt.show()   
