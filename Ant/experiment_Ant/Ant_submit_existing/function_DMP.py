import numpy as np
import math


def t2phi(t_max,theta,num_picture,k,i_update,num_update):
    tau=0.10
    
        
    alpha_c=tau*1.5
    theta_after=theta+0
        
    for i_theta in range(4):
        theta_before=theta[i_theta+20]+0
        while theta_before>=np.pi:
            theta_before=theta_before-2*np.pi
        while theta_before<=-np.pi:
            theta_before=theta_before+2*np.pi
        theta_after[i_theta+20]=theta_before+0
    
    
    phi_def1=np.array([theta_after[20]])
    phi_def2=np.array([theta_after[21]])
    phi_def6=np.array([theta_after[23]])
    phi_def3=phi_def2+np.array([theta_after[22]])
    phi_def4=phi_def2+phi_def6
    phi_def5=phi_def4+np.array([theta_after[22]])
    phi_def7=phi_def6+phi_def1
    
    
    phi_def=np.concatenate([np.array([0]),phi_def1,phi_def2,phi_def3,phi_def4,phi_def5,phi_def6,phi_def7],0)
    
    phi=np.zeros((8,t_max))
    
    for l_phi in range(8):
        
        phi_before=np.zeros(t_max)
        phi_ext=np.zeros(t_max)
        phi_ext_before=np.zeros(t_max)
        phi[l_phi,0]=0
        
        for t_i in range(t_max):
            t=t_i*0.01
            
            phi_ext_before[t_i]=phi_def[l_phi]+1/tau*t
            
            phi_ext[t_i]=phi_ext_before[t_i]+0
            while phi_ext[t_i]>=np.pi:
                phi_ext[t_i]=phi_ext[t_i]-2*np.pi
            while phi_ext[t_i]<=-np.pi:
                phi_ext[t_i]=phi_ext[t_i]+2*np.pi
                
            
            
            phi_before[t_i]=phi_ext_before[t_i]*(1-math.exp(-alpha_c/tau*t))
            
            phi[l_phi,t_i]=phi_before[t_i]+0
            while phi[l_phi,t_i]>=np.pi:
                phi[l_phi,t_i]=phi[l_phi,t_i]-2*np.pi
            while phi[l_phi,t_i]<=-np.pi:
                phi[l_phi,t_i]=phi[l_phi,t_i]+2*np.pi
                
    
    return phi,num_picture





def phi2position(phi,theta,r,t_max,L,num_picture):
    
    
    N=5
    
    c=np.zeros(N)
    h=np.zeros(N)
    for i in range(N):
        c[i]=2*np.pi/N*i
        h[i]=1

    x_up=np.zeros((L,t_max))
    x_down=np.zeros((L,t_max))
    x=np.zeros((L,t_max))
    
    for l_phi in range(L):
        phi_l=phi[l_phi,:]+0
        
        if l_phi<=3:
            omega=theta[5*(l_phi):5*(l_phi+1)]
        elif l_phi==4:
            omega=theta[10:15]
        elif l_phi==5:
            omega=theta[15:20]
        elif l_phi==6:
            omega=theta[0:5]
        elif l_phi==7:
            omega=theta[5:10]
        
        PHI=np.zeros((N,t_max))
        
        for t_i in range(t_max):
            for i in range(N):
                
                PHI[i,t_i]=math.exp(h[i]*math.cos(phi_l[t_i]-c[i])-1)    
            
            for i in range(N):
                
                x_up[l_phi,t_i]=x_up[l_phi,t_i]+PHI[i,t_i]*omega[i]
                x_down[l_phi,t_i]=x_down[l_phi,t_i]+PHI[i,t_i]
            x[l_phi,t_i]=x_up[l_phi,t_i]/x_down[l_phi,t_i]*r
            
            
        x_absmax=np.max([abs(np.min(x[l_phi,:])),abs(np.max(x[l_phi,:]))])
        if x_absmax>1:
            r=r/x_absmax
            
        
        x_up[l_phi,:]=np.zeros(t_max)
        x_down[l_phi,:]=np.zeros(t_max)
        x[l_phi,:]=np.zeros(t_max)
            
        for t_i in range(t_max):
            
            for i in range(N):
                
                x_up[l_phi,t_i]=x_up[l_phi,t_i]+PHI[i,t_i]*omega[i]
                x_down[l_phi,t_i]=x_down[l_phi,t_i]+PHI[i,t_i]
            x[l_phi,t_i]=x_up[l_phi,t_i]/x_down[l_phi,t_i]*r
                       
        if l_phi==4 or l_phi==6:
            x[l_phi,:]=-x[l_phi,:]+0
        
    position_target=x+0    
               
    
    return position_target,num_picture





def PDcontrol(position_target,state,state_before,t_i):
    
    L=position_target.shape[0]
    x_t=position_target[:,t_i]+0
    if t_i!=0:
        x_t_before=position_target[:,t_i-1]+0
    else:
        x_t_before=x_t+0
        
    x_pos=state[8:23][0::2]
    x_pos_before=state_before[8:23][0::2]
    feet_contact=state[24:28]
    
    position=np.zeros(L)
    action=np.zeros(L)
    for l in range(L):        
        
        if int(feet_contact[int(l/2)])==0:
            K_p=1.8
            K_d=5
        else:
            K_p=3.2
            K_d=5
        
        position[l]=x_pos[l]+0
        
        action[l]=K_p*(x_t[l]-x_pos[l])+K_d*((x_t[l]-x_pos[l])-(x_t_before[l]-x_pos_before[l]))
        
    
    return action,position,feet_contact

