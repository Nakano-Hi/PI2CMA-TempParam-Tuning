import gym
import pybullet_envs
import pickle
import numpy as np
from tqdm import tqdm
import datetime

import Exploration_Phase as ep
import ant_parameter_update as Apu
import function_DMP as FD

# hyperparameter (h)
h=10


num_experiment=10

L=8
t_max=1000
num_experiment=10
K=5

k_theta=24
r=0.3


for i_experiment in range(num_experiment):
            
    [theta0,Sigma0]=[np.zeros(k_theta),np.eye(k_theta)*10] # initial mu, Sigma
    theta_test_initial=ep.pick_sample(K,theta0,Sigma0)
    
    
    theta=theta0+0
    Sigma=Sigma0+0
    
    
    num_update=int(num_experiment/K)
    
    
    theta_save=np.zeros((num_update,k_theta))
    Sigma_save=np.zeros((num_update,k_theta,k_theta))
    h_save=np.zeros(num_update)
        
    num_picture=0
    
    env =gym.make('AntBulletEnv-v0')
    env.reset()
    
        
    
    for i_update in tqdm(range(num_update)):
                
        if i_update==0:
            theta_test=theta_test_initial+0 
        
        else:
            theta_test=ep.pick_sample(K,theta,Sigma)
    
    
        rewards=np.ones((K,t_max))*np.inf
        for k in range(K) :
            phi,num_picture=FD.t2phi(t_max,theta_test[k,:],num_picture,k,i_update,num_update)
            
            position_target,num_picture=FD.phi2position(phi,theta_test[k,:],r,t_max,L,num_picture)
            
            state = env.reset()
            
            state_before=state+0
            
            for t_i in range(t_max):
                action,position,feet_contact = FD.PDcontrol(position_target,state,state_before,t_i)
                
                state_before=state+0
                
                state, reward_before, done, info = env.step(action)
                reward=np.sum(reward_before)
                                
                rewards[k,t_i]=-reward+0
                
        
        rewards_1=rewards+0
        rewards_1[np.isinf(rewards)] = 0
        rewards[np.isinf(rewards)] = rewards_1.max()        
        
        
        J=rewards+0
        S=Apu.sum_J2S(J)
        
        
        if i_update ==0:
            theta_test_set=theta_test
            S_set=S[:,0]
        else:
            theta_test_set=np.concatenate([theta_test_set,theta_test],0)
            S_set=np.concatenate([S_set,S[:,0]],0)  
            
        
                        
        
        P=Apu.S2P(S,h)
                
        theta_new_i=Apu.P_theta2theta_new(P,theta_test)
        theta_new=Apu.theta_new_i_2_theta_new(theta_new_i)  
        Sigma_new_i=Apu.P_theta2Sigma_new(P,theta_test,theta)
        Sigma_new=Apu.Sigma_new_i_2_Sigma_new(Sigma_new_i)
        Sigma=Sigma_new+0
        
        theta=theta_new+0
        
        theta_save[i_update,:]=theta+0
        Sigma_save[i_update,:,:]=Sigma+0
    
    
        
    env.close()
    
    
    
    dt_now = datetime.date.today()
    now = datetime.datetime.now()
    
    with open('Ant_h'+str(h)+'_K'+str(K)+'_num_experiment'+str(num_experiment)+'_'+str(dt_now)+'_'+str(now.hour)+'_'+str(now.minute)+'_'+str(i_experiment)+'.binaryfile','wb') as web:
        pickle.dump([theta0,Sigma0,theta_test_set,S_set,theta_save,Sigma_save,h_save,K,num_update,r],web)
         

