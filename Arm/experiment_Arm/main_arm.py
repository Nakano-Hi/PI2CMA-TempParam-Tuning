import numpy as np
import datetime
import pickle

np.random.seed(seed=0)

#%% import function
from function_for_PI2CMA import pick_sample,sum_J2S,S2P_h,S2P_Lambda,sum_P,P2theta,P2Sigma
from function_for_proposed_method import calc_best_lambda
from function_for_DMP import calc_DMP
from function_for_calculate_J import calculate_J



#%% parameter for experiment
def param_experiment():        
    # set t_max
    t_max=1
    
    # method (sw_method=0: existing method, sw_method=1: proposed method, sw_method=2: Constant temperture parameter )
    sw_method=1
    # hyperparameter(sw_method=0: h, sw_method=1: alpha, sw_method=2: Constant lambda )
    hyper_param=0.7
    
    
    
    tau=1
    Hz=50
    dt=1/Hz
    i_t_max=int(Hz*t_max)    
    
    # create PHI
    B=5
    
    # number of arm
    D=5
    
    # number of update
    num_update=30
    
    # population
    K=100
    
    # initial theta,Sigma
    theta0=np.zeros(D*B)
    Sigma0=np.eye(D*B,D*B)*10**6
    
    
    # the number of conducting experiment
    num_experiment=30

    return [tau,dt,t_max,i_t_max,B,D,num_update,K,theta0,Sigma0,sw_method,hyper_param,num_experiment]
        
[tau,dt,t_max,i_t_max,B,D,num_update,K,theta0,Sigma0,sw_method,hyper_param,num_experiment]=param_experiment()


        
#%% main

for i_experiment in range(num_experiment):
    
    # definition variable for save
    S_save=np.zeros((num_update,K))
    P_sum_save=np.zeros((num_update,K))
    P_save_all=np.zeros((num_update,K,i_t_max))
    theta_save=np.zeros((num_update,D*B))
    Sigma_save=np.zeros((num_update,D*B,D*B))
    Lambda_h_save=np.zeros(num_update)
    
    # set initial theta,Sigma
    theta=theta0.copy()
    Sigma=Sigma0.copy()
    
    # tentative value
    Lambda_before=10**7
    Lambda=0
    

    # load the initial policy parameter for experiment (theta_k)
    with open('arm_DMP_initial.binaryfile', 'rb') as web:
        [theta_test_initial]= pickle.load(web)      
    
    # set the initial policy parameter for experiment
    theta_test=theta_test_initial.copy()
    
    for i_update in range(num_update):
        
        if i_update!=0:
            # sampling
            theta_test=pick_sample(K,theta,Sigma)
        
        # experiment with DMP
        y_save=calc_DMP(theta_test,K,D,B,i_t_max,dt,tau)
        
        # calculate the cost
        J=calculate_J(y_save,dt,i_t_max,D)
        # caluculate the cost-to-go
        S=sum_J2S(J)
        
        print('experiment:'+str(i_experiment))
        print('update:'+str(i_update))
        
        # Parameter Update Phase
        if sw_method==0:
            # existing method
            h=hyper_param
            P=S2P_h(S,h)
            
        elif sw_method==1:
            # proposed method
            alpha=hyper_param
            Lambda=calc_best_lambda(K,theta_test,S,alpha,theta,Lambda_before)
            P=S2P_Lambda(S,Lambda)
            Lambda_before=Lambda+0
            
        else:
            # constant
            Lambda=hyper_param
            P=S2P_Lambda(S,Lambda)
            
        
        # caluculate the theta and Sigma for next update
        P_sum=sum_P(P)+0        
        theta_new=P2theta(theta_test,P_sum)+0
        Sigma_new=P2Sigma(theta_test,P_sum,theta)+0
        
        # update theta and Sigma
        theta=theta_new+0
        Sigma=Sigma_new+0
        
        
        
        
        # save the result
        S_save[i_update,:]=S[:,0]
        P_sum_save[i_update,:]=P_sum.reshape(1,P_sum.shape[0])
        P_save_all[i_update,:,:]=P+0
        theta_save[i_update,:]=theta+0
        Sigma_save[i_update,:,:]=Sigma+0
        Lambda_h_save[i_update]=Lambda
        
        if sw_method==0:
            Lambda_h_save[i_update]=h
            
    
    
    if True:
        # save the results by ``pickle''
        
        dt_now = datetime.date.today()
        now = datetime.datetime.now()
        if sw_method==0:
            with open('arm_submit_'+str(t_max)+'sec_h'+str(h)+'_'+str(dt_now)+'_'+str(now.hour)+'_'+str(now.minute)+str(i_experiment)+'.binaryfile','wb') as web:
                pickle.dump([S_save,P_sum_save,P_save_all,theta_save,Sigma_save,Lambda_h_save,theta_test_initial],web)
        elif sw_method==1:
            with open('arm_submit_'+str(t_max)+'sec_a'+str(alpha)+'_'+str(now.hour)+'_'+str(now.minute)+str(i_experiment)+'.binaryfile','wb') as web:
                pickle.dump([S_save,P_sum_save,P_save_all,theta_save,Sigma_save,Lambda_h_save,theta_test_initial],web)
        else:
            with open('arm_submit_'+str(t_max)+'sec_Lconst'+str(Lambda)+'_'+str(now.hour)+'_'+str(now.minute)+str(i_experiment)+'.binaryfile','wb') as web:
                pickle.dump([S_save,P_sum_save,P_save_all,theta_save,Sigma_save,Lambda_h_save,theta_test_initial],web)
            

