

import numpy as np

NUM_MEASUREMENTS = 4

# TODO Part 3: Comment the code explaining each part
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        # assuming P, Q, R, are scalar, therefore need to multiply them by corresponding identity matrices to get covariance matrices
        self.P = P # initial prediction (prior) covariance - is nxn, n is num of states
        self.Q = Q # covariance of the states (assumed constant) - is nxn
        self.R = R # covariance of measurements (assumed constant) - is bxb, b is num of measurements
        self.x = x # initial state
        self.dt = dt
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):
        # computes prior (prediction mean and covariance) using linearization of A about previous state mean
        # A describes non linear motion model
        # C describes non linear mapping from state to measurement
        self.A = self.jacobian_A() # linearization of A about previous state mean
        

        # Q: shouldnt C be calculated after prediction mean calculated since it needs current prediction mean (as per lec notes)?

        #calculates prediction mean (updates x internally)
        self.motion_model()
        self.C = self.jacobian_H()
        #calc jacobian_H after updating prediction mean as jacobian_H requires prediction mean of current time step
         #linearization of C about prediction mean
        #calculates prediction covariance using jacobian of A
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):
        # computes posterior (posterior mean and variance) using prediction and current measurements
        # uses linearization of C about current prediction mean
        # A and C already updated as jacobians in predict
        # S matrix is "denominator" (expression that is inversed) of kalman gain
        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        
        # kalman gain used to compute state estimate mean and covariance based
        # qualitatively tells you if you trust odom or sensor (IMU) more
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        surprise_error= z - self.measurement_model()
        # updated state estimate mean (accounting for measurements)
        # note the state estimate (x) just becomes in the state estimate mean (mean of posterior)
        # since the mean value has the highest probability since EKF assumes gaussian distributions
        self.x=self.x + np.dot(kalman_gain, surprise_error)
        # updated state estimate covariance (accounting for measurements)
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax - vdot in x dir in robot frame
            v*w, # ay - using formula for centripetal motion ay = v^2/r, and r = v/w
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        # returns next state given previous state using motion model
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        #motion model describes how to get from current to next state only considering past state and input
        self.x = np.array([
            x + v * np.cos(th) * dt, #these are global x and y, so must add v*cos(th)*dt and v*sin(th)*dt to x and y respectively
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        #jacobian_A computes the linearization of A about the previous state mean
        # Q: do we even technically have inputs? bc v and w part of state vector?
        #taking partial derivatives of motion model equations with respect to previous state variables and inputs (parameters of G)
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,             -v*np.sin(th)*dt, 0,   np.cos(th)*dt,  0],
            [0, 1,              v*np.cos(th)*dt, 0,   np.sin(th)*dt,  0],
            [0, 0,                1, dt,              0,  0],
            [0, 0,                0, 1,               0,  0],
            [0, 0,                0, 0,               1,  dt],
            [0, 0,                0, 0,               0,  1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        #jacobian_H computes the linearization of C about the current prediction state mean
        #taking partial derivaties of measurement equations with respect to state variables (inputs of H)
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
