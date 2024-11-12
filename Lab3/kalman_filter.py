

import numpy as np



# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        # assign variables to self
        self.P= P
        self.Q= Q
        self.R= R
        self.x= x
        self.dt = dt
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):
        # set jacobians
        self.A = self.jacobian_A() # nxn matrix, evolves state from k-1 to k
        self.C = self.jacobian_H() # bxn matrix that maps state to observation 
        # not sure if this is right but H and C seem to be the same?
        
        self.motion_model() # predicts future values with given values
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q # predicts future step
        #based on transition matrix A and prediction matrix P. Q represents added noise

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):

        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R # for computing kalman gain
            
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S)) # find how much input measurement will influence state estimate

        surprise_error= z - self.measurement_model() # used to estimate system error
        
        self.x=self.x + np.dot(kalman_gain, surprise_error) # update belief with measurements
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P) #update covariance matrix to incorporate certainty from past prediction
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot*np.cos(th), # ax
            vdot*np.sin(th), # ay
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt, #should be v_(k-1), is this prev timestep?
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    
    def jacobian_A(self): #nxn matrix that describes how state evolves from k-1 to k (no controls, noise)
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([ #just a bunch of partial derivatives
            #x, y,               th, w,             v, vdot
            [1, 0, -v*np.sin(th)*dt, 0,np.cos(th)*dt,  0],
            [0, 1,  v*np.cos(th)*dt, 0,np.sin(th)*dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self): #second derivative matrix
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x