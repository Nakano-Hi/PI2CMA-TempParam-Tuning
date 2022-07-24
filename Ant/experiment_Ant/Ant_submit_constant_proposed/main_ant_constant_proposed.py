import gym
import pybullet_envs
import pickle
import numpy as np
from tqdm import tqdm
import datetime

import Exploration_Phase as ep
import ant_parameter_update as Apu
import function_DMP as FD
import function_proposed_method as FPM


sw_Lambda=True # if True:proposed method, if False: Constant lambda
hyper_param=0.8
num_experiment=10

# the number of time step for experiment
t_max=1000

num_update=1
K=100
k_theta=24

Lambda=10000 # tentative data


for i_experiment in range(num_experiment):
        
    if sw_Lambda==True:
        # proposed method
        alpha=hyper_param+0        
    else:
        # existing method
        Lambda=hyper_param+0  
            
    r=0.3 # parameter of amplitude used by DMP
    [theta0,Sigma0]=[np.zeros(k_theta),np.eye(k_theta)*10] # initial mu, Sigma
    theta_test_initial=ep.pick_sample(K,theta0,Sigma0)    
    
    theta=theta0+0
    Sigma=Sigma0+0
    
        
    # tentative value
    theta_save=np.zeros((num_update,k_theta))
    Sigma_save=np.zeros((num_update,k_theta,k_theta))
    
    Lambda_save=np.zeros(num_update)
    
    # Ant initial setting
    env =gym.make('AntBulletEnv-v0')
    env.reset()
    
            
    #%% update start
    for i_update in tqdm(range(num_update)):                
        # exploration phase
        if i_update==0:
            # use the same initial policy parameters 
            theta_test=theta_test_initial+0 
        else:
            # generate policy parameters 
            theta_test=ep.pick_sample(K,theta,Sigma)
    
        J=np.zeros((K,t_max)) # tentative value
        
        #%% experiment and calculating cost        
        for k in range(K) :
            
            # calculate phi used by DMP
            phi=FD.t2phi(t_max,theta_test[k,:],k,i_update,num_update)
            
            # calculate the target position
            position_target=FD.phi2position(phi,theta_test[k,:],r,t_max)
            
            state = env.reset()
                        
            state_before=state.copy() # tentative value
            
            for t_i in range(t_max):
                action,position,feet_contact = FD.PDcontrol(position_target,state,state_before,t_i)
                
                state_before=state.copy() # save for D control
                
                state, reward_before, done, info = env.step(action)
                reward=np.sum(reward_before)
                                
                J[k,t_i]=-reward
                
        
        #%% Parameter Update Phase      
        
        # calculate cost-to-go(S) from cost(J)
        S=Apu.sum_J2S(J)
        
        
        # save the cost-to-go
        if i_update ==0:
            theta_test_set=theta_test
            S_set=S[:,0]
        else:
            theta_test_set=np.concatenate([theta_test_set,theta_test],0)
            S_set=np.concatenate([S_set,S[:,0]],0)  
            
        
        
        #%% select the temperture parameter by proposed method
        if sw_Lambda:
            Lambda=FPM.calc_best_lambda(K,theta_test,k_theta,S,alpha,theta)                    
            
        Lambda_save[i_update]=Lambda+0                
        
        
        #%% Parameter Update Phase    
        # S → P
        P=Apu.S2P(S,Lambda)
                
        # theta_new_i
        theta_new_i=Apu.P_theta2theta_new(P,theta_test)
        # theta_new
        theta_new=Apu.theta_new_i_2_theta_new(theta_new_i)        
    
        # Sigma_new_i
        Sigma_new_i=Apu.P_theta2Sigma_new(P,theta_test,theta)
        # Sigma_new
        Sigma_new=Apu.Sigma_new_i_2_Sigma_new(Sigma_new_i)
        Sigma=Sigma_new+0
        
        #update mu, Sigma
        theta=theta_new+0
        
        theta_save[i_update,:]=theta+0
        Sigma_save[i_update,:,:]=Sigma+0
    
    
        
    env.close()
    
    
    
    dt_now = datetime.date.today()
    now = datetime.datetime.now()
    
    if True:        
        
        if sw_Lambda:
            with open('Ant_proposed_alpha'+str(alpha)+'_K'+str(K)+'_'+str(dt_now)+'_'+str(now.hour)+'_'+str(now.minute)+'_'+str(i_experiment)+'.binaryfile','wb') as web:
                pickle.dump([theta0,Sigma0,theta_test_set,S_set,theta_save,Sigma_save,Lambda_save,K,num_update,r],web)
                
        else:
            with open('Ant_constant_Lambda'+str(Lambda)+'_K'+str(K)+'_'+str(dt_now)+'_'+str(now.hour)+'_'+str(now.minute)+'_'+str(i_experiment)+'.binaryfile','wb') as web:
                pickle.dump([theta0,Sigma0,theta_test_set,S_set,theta_save,Sigma_save,Lambda_save,K,num_update,r],web)
                 




