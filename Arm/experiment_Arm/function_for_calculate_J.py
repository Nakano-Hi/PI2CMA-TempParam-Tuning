import numpy as np
import numpy.linalg as LA


#%% calculate J
def calculate_J(y_save,dt,i_t_max,D):
    [position,acc]=calculate_position(y_save,dt,i_t_max,D)
    
    J=np.zeros((position.shape[0],position.shape[2]))
    K=position.shape[0]
    D=position.shape[1]
    for k in range(K):
        for i_t in range(position.shape[2]):
            J_D=np.zeros(D)
            for i_D in range(D):
                J_D[i_D]=(D-i_D)/np.sum(np.arange(D)+1)*(acc[k,i_D,i_t])**2
            
            # angular acceleration cost
            J[k,i_t]=sum(J_D)                
            
            # distanse cost at t=0.3
            if i_t==14:
                J[k,i_t]=J[k,i_t]+LA.norm(position[k,position.shape[1]-1,i_t,:]-np.array([0.5,0.5]))**2*10**8
    return J

#%% calculate the position of arm tip
def calculate_position(y_save_extract,dt,i_t_max,D):
    
    # calculate the position for each joint
    position=np.zeros((y_save_extract.shape[0],y_save_extract.shape[1],y_save_extract.shape[2],2))
    for k in range(y_save_extract.shape[0]):        
        for i_t in range(i_t_max):
            y_sum=np.concatenate([np.array([np.sum(y_save_extract[k,:i_d+1,i_t]) for i_d in range(D)]),np.array([np.sum([y_save_extract[k,:,i_t]])])])
        
            for i_D in range(D):
                if i_D==0:                
                    position[k,i_D,i_t,0]=1/D*np.cos(y_sum[i_D])
                    position[k,i_D,i_t,1]=1/D*np.sin(y_sum[i_D])
                else:
                    position[k,i_D,i_t,0]=position[k,i_D-1,i_t,0]+1/D*np.cos(y_sum[i_D])
                    position[k,i_D,i_t,1]=position[k,i_D-1,i_t,1]+1/D*np.sin(y_sum[i_D])
    
    # calculate the angular acceleration for each joint
    acc=np.zeros((y_save_extract.shape[0],y_save_extract.shape[1],y_save_extract.shape[2]))
    for k in range(y_save_extract.shape[0]):
        for i_t in range(i_t_max):
            for i_D in range(D):
                if i_t>=2:
                    acc[k,i_D,i_t]=(y_save_extract[k,i_D,i_t]-y_save_extract[k,i_D,i_t-1]*2+y_save_extract[k,i_D,i_t-2])/dt**2
                elif i_t==1:
                    acc[k,i_D,i_t]=(y_save_extract[k,i_D,i_t]-y_save_extract[k,i_D,i_t-1]*2)/dt**2
                else:
                    acc[k,i_D,i_t]=(y_save_extract[k,i_D,i_t])/dt**2
    return [position,acc]
