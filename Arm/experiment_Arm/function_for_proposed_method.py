import numpy as np
import numpy.linalg as LA
from scipy.spatial import distance

from function_for_PI2CMA import pick_sample,sum_J2S,S2P_h,S2P_Lambda,sum_P,P2theta,P2Sigma

# calculation of F,V
def calculate_FV(theta_test,theta,P_sum,theta_old,Sigma_old,R):
    
    # calculate theta for next step
    theta_lambda=P2theta(theta_test,P_sum) 
    # calculate Sigma for next step
    Sigma_lambda=P2Sigma(theta_test,P_sum,theta)
    
    theta_r=pick_sample(R,theta_lambda,Sigma_lambda)
    
    # portion of f_old(theta)
    exp_old=np.array([np.exp(-1/2*distance.mahalanobis(theta_r[k,:], theta_old, LA.inv(Sigma_old))**2) for k in np.arange(R)])
    # portion of f_lambda(theta)
    exp_lambda=np.array([np.exp(-1/2*distance.mahalanobis(theta_r[k,:], theta_lambda, LA.inv(Sigma_lambda))**2) for k in np.arange(R)])
    
    # calculate log(det(Sigma_old))
    sgn_V_sigma_old,ln_det_Sigma_old=LA.slogdet(Sigma_old)
    # calculate log(det(Sigma_lambda))
    sgn_V_sigma,ln_det_Sigma_lambda=LA.slogdet(Sigma_lambda)
    
    # calculate f_old / f_lambda
    f_oldPARnew=np.exp(1/2*(ln_det_Sigma_lambda-ln_det_Sigma_old))*exp_old/exp_lambda
    
    # if f_old / f_lambda>=1, change to f_old / f_lambda=1 (representing the max((f_lambda-f_old)/f_lambda),0)
    for k in range(f_oldPARnew.shape[0]):
        if f_oldPARnew[k]>=1:
            f_oldPARnew[k]=1
    
    # calculate F
    F=1-np.average(f_oldPARnew)
    # calculate log(V)
    ln_V=ln_det_Sigma_lambda+np.log(F)
    
    return [F,ln_V]



def calc_best_lambda(K,theta_test,S,alpha,theta,Lambda_before):
    candidate_lambda=100
    ratio_lambda=0.95
    
    # the number of sampling
    R=2000
    
    # find best lambda around lambda using before update
    Lambda=Lambda_before*1.2+0
    
    # calculate theta_old, Sigma_old
    P_old=np.ones(K).reshape(K,1)/K
    theta_old=np.sum(theta_test*P_old,axis=0)
    Sigma_old=sum([P_old[k,0]*np.outer((theta_test[k,:]-theta), (theta_test[k,:]-theta)) for k in np.arange(K)])    

    
    # decrease the lambda until F is lower than alpha
    # If F is upper than alpha, save the lambda whose V is largest. 
    ln_V_before=-9999    
    i_initial=0
    for i_Lambda in range(candidate_lambda):            
        P_Lambda=S2P_Lambda(S,Lambda)
        P_Lambda_sum=sum_P(P_Lambda)
        [F,ln_V]=calculate_FV(theta_test,theta,P_Lambda_sum,theta_old,Sigma_old,R)
        
        print('lambda:'+str(int(Lambda))+', F:'+str(round(F,4)))
        
        if i_initial==0:
            if alpha<=F:
                Lambda=Lambda*2
                continue
            
        if alpha<=F:
            if ln_V>ln_V_before:
                Lambda_best=Lambda+0
                ln_V_before=ln_V+0
                count_FV=0
            else:
                count_FV=count_FV+1
                if count_FV>=3:
                    break
        
        Lambda=Lambda*ratio_lambda
        
        i_initial=1
        
    print('determine lambda:'+str(round(Lambda_best,4)))

    return Lambda_best

