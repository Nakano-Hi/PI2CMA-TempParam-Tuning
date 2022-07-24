import numpy as np
import pandas as pd


def pick_sample(K,theta,Sigma):
    theta_list = theta.tolist()
    Sigma_list = Sigma.tolist()
    theta_test=np.random.multivariate_normal(theta_list, Sigma_list, K)
    return theta_test


def sum_J2S(J):
    
    S=np.zeros((J.shape[0],J.shape[1]))
    for k in range(J.shape[0]):
        for n in range(J.shape[1]):
            S[k,n]=sum(J[k][n:J.shape[1]])
    return S

# calculate P using existing method
def S2P_h(S,h):
    S_def=S-np.min(S,axis=0)
    x=S_def/np.max(S_def,axis=0)
    exp_xh=np.exp(-h*x)
    P=exp_xh/np.sum(exp_xh,axis=0)
    return P


# calculate P using proposed method and Constant lambda. 
def S2P_Lambda(S,Lambda):
    exp_x=np.exp(-S/Lambda)
    P=exp_x/np.sum(exp_x,axis=0)
    
    for n in range(P.shape[1]):            
        df_Pn = pd.DataFrame(P[:,n])  
        
        # while P contains nan, recalculate P. 
        i_nan=0
        while df_Pn.isnull().values.sum()>0:
            i_nan=i_nan+1
            df_Pn = pd.DataFrame(P[:,n])    
            exp_xn=np.exp(-S[:,n]/Lambda+0.01*i_nan)
            P[:,n]=exp_xn/np.sum(exp_xn,axis=0)
        
    return P


# calclate the sum(P)
def sum_P(P):
    P_sum=np.zeros(P.shape[0])
    for k in range(P.shape[0]):        
        P_sum[k]=np.sum([P[k,i]*(P.shape[1]-i) for i in range(P.shape[1])])
    P_sum=P_sum/np.sum([P.shape[1]-i for i in range(P.shape[1])])
    return P_sum.reshape(P_sum.shape[0],1)

# calculate the theta for next step from P
def P2theta(theta_test,P_sum):
    theta_new=np.sum(theta_test*P_sum,axis=0)
    return theta_new
    
# calculate the Sigma for next step from P
def P2Sigma(theta_test,P,theta):
    Sigma_new=np.zeros((theta_test.shape[1],theta_test.shape[1]))
    K=theta_test.shape[0]
    for k in range(K):
        Sigma_new=Sigma_new+P[k,0]*np.outer((theta_test[k,:]-theta), (theta_test[k,:]-theta))
    return Sigma_new

