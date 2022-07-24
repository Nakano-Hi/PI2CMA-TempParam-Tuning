
import numpy as np


#%% calculate the trajectory of the joint angles by using DMP
def calc_DMP(theta_test,K,D,B,i_t_max,dt,tau):
    
    [x,g]=canonical_optional_system(i_t_max,dt,tau,D)
    PHI=calc_PHI(x,B,i_t_max)
    
    y_save=np.zeros((K,D,i_t_max))

    for k in range(K):
        theta_k=theta_test[k,:]
        for i_D in range(D):
            [y_before,z_before]=[0,0]
            omega=theta_k[B*i_D:B*(i_D+1)]
            
            for i_t in range(i_t_max):     
                PHI_f=PHI[:,i_t]
                x_f=x[i_t]
                
                f=forcing_system(omega,PHI_f,g[i_t],B,x_f)
                [y,z]=transformation_system(g[i_t],y_before,z_before,f,dt,tau)
                [y_before,z_before]=[y,z]
                y_save[k,i_D,i_t]=y

    return y_save


# parameter for DMP
def param_DMP():        
    a_z=25
    b_z=a_z/4
    a_x=a_z/3
    a_g=a_z/2
    y0=0
    return [a_z,b_z,a_x,a_g,y0]


#%% diferrential equation for DMP

def forcing_system(omega,PHI_f,g_f,B,x_f):
    [a_z,b_z,a_x,a_g,y0]=param_DMP()
    f_up=0
    f_down=0    
    for i_B in range(B):
        f_up=f_up+PHI_f[i_B]*omega[i_B]
        f_down=f_down+PHI_f[i_B]
    f=f_up/f_down*x_f*(g_f-y0)
    return f


def transformation_system(g_t,y_before,z_before,f,dt,tau):
    [a_z,b_z,a_x,a_g,y0]=param_DMP()
    y=y_before+dt/tau*z_before
    z=((tau-a_z*dt)*z_before+a_z*b_z*dt*(g_t-y_before)+f*dt)/tau
    return [y,z]


def canonical_optional_system(i_t_max,dt,tau,D):
    [a_z,b_z,a_x,a_g,y0]=param_DMP()
        
    # canonical system
    x=np.zeros(i_t_max)
    x[0]=1
    for i_t in range(i_t_max-1):    
        x[i_t+1]=x[i_t]*(1-a_x*dt/tau)
        
    # optional terms
    for a in range(1000):    
        sum_A=np.sum([np.cos((d+1)*a/1000*np.pi/2) for d in range(D)])
        if sum_A<0:
            a_result=a-1
            g0_result=a_result/1000*np.pi/2
            break
        
    g0=g0_result+0
    g=np.zeros(i_t_max)
    for i_t in range(i_t_max-1):    
        g[i_t+1]=g[i_t]*(1-a_g*dt/tau)+a_g*dt/tau*g0
        
    return [x,g]
        
        
def calc_PHI(x,B,i_t_max):
        
    c=np.zeros(B)
    h=np.zeros(B)
    for i_B in range(B):
        c[i_B]=1/B*(i_B+1)
        h[i_B]=1
    PHI=np.zeros((B,i_t_max))
    for i_t in range(i_t_max):          
        for i_B in range(B):
            PHI[i_B,i_t]=np.exp(-h[i_B]*(x[i_t]-c[i_B])**2)
                        
    return PHI
            
            