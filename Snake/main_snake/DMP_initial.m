% set the parameter using DMP
function paramDMP=DMP_initial(paramRL)
    paramDMP.M=paramRL.M;
    paramDMP.L=paramRL.L;
    
    % parameter of DMP
    paramDMP.tau=0.2;
    paramDMP.a_z=25;
    paramDMP.b_z=paramDMP.a_z/4;
    paramDMP.a_r=25/2;
    paramDMP.a_g=25/2;    
    
    paramDMP.c=0:2*pi/paramDMP.M:2*pi/paramDMP.M*(paramDMP.M-1);
    paramDMP.h=ones(1,paramDMP.M);
    
    paramDMP.r0=paramDMP.a_z*paramDMP.b_z*pi/3;    
    paramDMP.g0=0;    
    paramDMP.y0=zeros(1,paramDMP.L);
    paramDMP.z0=zeros(1,paramDMP.L);
    paramDMP.phi0=0;
    
    % devide dt into N_dt parts to improve calculation accuracy of solving differential equation
    paramDMP.N_dt=10;
    
    % tentative parameter for solving differential equation
    paramDMP.y_before=paramDMP.y0;
    paramDMP.z_before=paramDMP.z0;
    paramDMP.r_before=paramDMP.r0;
    paramDMP.g_before=paramDMP.g0;
    paramDMP.phi_before=paramDMP.phi0;
    
end
