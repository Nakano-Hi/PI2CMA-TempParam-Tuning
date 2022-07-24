import numpy as np

def sum_J2S(J):
    
    S=np.zeros((J.shape[0],J.shape[1]))
    for k in range(J.shape[0]):
        for n in range(J.shape[1]):
            S[k,n]=sum(J[k][n:J.shape[1]])
    return S



def S2P(S,h):
    S_def=S-np.min(S,axis=0)
    x=S_def/np.max(S_def,axis=0)
    exp_xh=np.exp(-h*x)
    P=exp_xh/np.sum(exp_xh,axis=0)
    return P


def P_theta2theta_new(P,theta_test):
    K=P.shape[0]
    N=P.shape[1]
    P_theta=np.zeros((K,N,theta_test.shape[1]))
    theta_new_i=np.zeros((N,theta_test.shape[1]))
    
    for n in range(N):
        for k in range(K):
            P_theta[k,n,:]=P[k,n]*theta_test[k,:]
        for m in range(theta_test.shape[1]):
            theta_new_i[n,m]=sum(P_theta[:,n,m])
    return theta_new_i


def theta_new_i_2_theta_new(theta_new_i):
    N=theta_new_i.shape[0]
    theta_up=np.zeros((N,theta_new_i.shape[1]))
    theta_down=np.zeros(N)
    theta_up_sum=np.zeros(theta_new_i.shape[1])
    theta_new=np.zeros(theta_new_i.shape[1])
    
    for m in range(theta_new_i.shape[1]):
        for n in range (N):
            theta_up[n,m]=(N-n)*theta_new_i[n,m]
            theta_down[n]=N-n
        theta_up_sum[m]=sum(theta_up[:,m])
        theta_down_sum=sum(theta_down)
        theta_new[m]=theta_up_sum[m]/theta_down_sum
    return theta_new


def P_theta2Sigma_new(P,theta_test,theta):
    K=P.shape[0]
    N=P.shape[1]
    P_theta_theta=np.zeros((K,N,theta_test.shape[1],theta_test.shape[1]))
    Sigma_new_i=np.zeros((N,theta_test.shape[1],theta_test.shape[1]))
    for n in range(N):
        for k in range(K):
            P_theta_theta[k,n,:,:]=P[k,n]*np.outer((theta_test[k,:]-theta), (theta_test[k,:]-theta))
        for m in range(theta_test.shape[1]):
            for l in range(theta_test.shape[1]):
                    Sigma_new_i[n,m,l]=sum(P_theta_theta[:,n,m,l])
    return Sigma_new_i


def Sigma_new_i_2_Sigma_new(Sigma_i):
    N=Sigma_i.shape[0]
    Sigma_up=np.zeros((N,Sigma_i.shape[1],Sigma_i.shape[2]))
    Sigma_down=np.zeros(N)
    Sigma_up_sum=np.zeros((Sigma_i.shape[1],Sigma_i.shape[2]))
    Sigma_new=np.zeros((Sigma_i.shape[1],Sigma_i.shape[2]))
    
    for m in range(Sigma_i.shape[1]):
        for l in range(Sigma_i.shape[2]):
            for n in range(N):
                Sigma_up[n,m,l] = (N-n)*Sigma_i[n,m,l]
                Sigma_down[n]=N-n
            Sigma_up_sum[m,l]=sum(Sigma_up[:,m,l])
            Sigma_down_sum=sum(Sigma_down)
            Sigma_new[m,l]=Sigma_up_sum[m,l]/Sigma_down_sum
    return Sigma_new
    



