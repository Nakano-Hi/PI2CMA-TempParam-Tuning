import numpy as np
import numpy.linalg as LA
from scipy.spatial import distance

import Exploration_Phase as Ep
import ant_parameter_update as Apu





def calc_best_lambda(K,theta_test,k_theta,S,alpha,theta):
    K_amp=50
    K_Lambda=K*K_amp
    
    P_Lambda_old=np.ones(K).reshape(K,1)/K
    theta_Lambda_old=np.sum(theta_test*P_Lambda_old,axis=0)
    Sigma_Lambda_old=sum([P_Lambda_old[k,0]*np.outer((theta_test[k,:]-theta), (theta_test[k,:]-theta)) for k in np.arange(K)])    
    sgn_V_sigma_old,ln_det_Sigma_lambda_max_old=LA.slogdet(Sigma_Lambda_old)
    
    Lambda_f=100
    
    ratio=0.95
    
    for i in range(200): 
        P_Lambda=Apu.S2P(S,Lambda_f)
        
        theta_Lambda_i=Apu.P_theta2theta_new(P_Lambda,theta_test)
        theta_Lambda=Apu.theta_new_i_2_theta_new(theta_Lambda_i)        
    
        Sigma_Lambda_i=Apu.P_theta2Sigma_new(P_Lambda,theta_test,theta)
        Sigma_Lambda=Apu.Sigma_new_i_2_Sigma_new(Sigma_Lambda_i)
        theta_Lambda_test=Ep.pick_sample(K_Lambda,theta_Lambda,Sigma_Lambda)
        sgn_V_sigma,ln_det_Sigma_lambda_max=LA.slogdet(Sigma_Lambda)
        
        exp_old=np.array([np.exp(-1/2*distance.mahalanobis(theta_Lambda_test[k,:], theta_Lambda_old, LA.pinv(Sigma_Lambda_old))**2) for k in np.arange(K_Lambda)])
        exp_Lambda=np.array([np.exp(-1/2*distance.mahalanobis(theta_Lambda_test[k,:], theta_Lambda, LA.pinv(Sigma_Lambda))**2) for k in np.arange(K_Lambda)])
                
        f_oldPARnew=np.exp(1/2*(ln_det_Sigma_lambda_max-ln_det_Sigma_lambda_max_old))*exp_old/exp_Lambda
        for k in range(f_oldPARnew.shape[0]):
            if f_oldPARnew[k]>=1:
                f_oldPARnew[k]=1
    
        if 1-np.average(f_oldPARnew)>alpha:
            Lambda_f=Lambda_f/ratio
            break
        
        Lambda_f=Lambda_f*ratio
        
        
    Lambda_v=Lambda_f+0
    
    num_Lambda=100
    ln_det_Sigma_lambda=np.ones(num_Lambda)*(-9999)
    lambda_candidate=np.zeros(num_Lambda)
    ln_det_Sigma_lambda_max_def_save=np.zeros(num_Lambda)
    
    
    count_skip=0
    for i in range(num_Lambda): 
        
        P_Lambda=Apu.S2P(S,Lambda_v)
        
        theta_Lambda_i=Apu.P_theta2theta_new(P_Lambda,theta_test)
        theta_Lambda=Apu.theta_new_i_2_theta_new(theta_Lambda_i)        
    
        Sigma_Lambda_i=Apu.P_theta2Sigma_new(P_Lambda,theta_test,theta)
        Sigma_Lambda=Apu.Sigma_new_i_2_Sigma_new(Sigma_Lambda_i)
        
        
        theta_Lambda_test=Ep.pick_sample(K_Lambda,theta_Lambda,Sigma_Lambda)
        
        sgn_V_sigma,ln_det_Sigma_lambda_max=LA.slogdet(Sigma_Lambda)
        ln_det_Sigma_lambda_max_def_save[i]=ln_det_Sigma_lambda_max+0
        
        lambda_candidate[i]=Lambda_v+0
        
        Lambda_v=Lambda_v*0.95
        
        if ln_det_Sigma_lambda_max<np.max(ln_det_Sigma_lambda):
            count_skip=count_skip+1
            if count_skip>3:
                break
            
            continue
        else:
            count_skip=0
        
        exp_old=np.array([np.exp(-1/2*distance.mahalanobis(theta_Lambda_test[k,:], theta_Lambda_old, LA.pinv(Sigma_Lambda_old))**2) for k in np.arange(K_Lambda)])
        exp_Lambda=np.array([np.exp(-1/2*distance.mahalanobis(theta_Lambda_test[k,:], theta_Lambda, LA.pinv(Sigma_Lambda))**2) for k in np.arange(K_Lambda)])
        
        
        f_oldPARnew=np.exp(1/2*(ln_det_Sigma_lambda_max-ln_det_Sigma_lambda_max_old))*exp_old/exp_Lambda
        for k in range(f_oldPARnew.shape[0]):
            if f_oldPARnew[k]>=1:
                f_oldPARnew[k]=1
        if np.average(f_oldPARnew)>=1:
            continue
        
        ln_det_Sigma_lambda[i]=ln_det_Sigma_lambda_max+np.log(1-np.average(f_oldPARnew))
        

    Lambda_new=lambda_candidate[np.argmax(ln_det_Sigma_lambda)]

    return Lambda_new

