import numpy as np

def pick_sample(K,theta,Sigma):
    theta_list = theta.tolist()
    Sigma_list = Sigma.tolist()
    theta_test=np.random.multivariate_normal(theta_list, Sigma_list, K)
    return theta_test
