
% calculate the reference angle for each joint by DMP
function paramDMP=calc_DMP(paramDMP,theta_k,dt)
    if dt==0
       paramDMP=DMP_rhysmic(paramDMP,0,theta_k);

       paramDMP.y_before=paramDMP.y;
       paramDMP.z_before=paramDMP.z;
       paramDMP.r_before=paramDMP.r;
       paramDMP.g_before=paramDMP.g;
       paramDMP.phi_before=paramDMP.phi;
        
    else
        dt_j=dt/paramDMP.N_dt;

        for n_dt=1:paramDMP.N_dt
           paramDMP=DMP_rhysmic(paramDMP,dt_j,theta_k);

           paramDMP.y_before=paramDMP.y;
           paramDMP.z_before=paramDMP.z;
           paramDMP.r_before=paramDMP.r;
           paramDMP.g_before=paramDMP.g;
           paramDMP.phi_before=paramDMP.phi;
        end
    end
end


% differential equation of DMP
function paramDMP=DMP_rhysmic(paramDMP,dt_j,theta_k)
    
    % phi
    paramDMP.phi=paramDMP.phi_before+(1/paramDMP.tau)*dt_j;
    
    % g
    paramDMP.g=paramDMP.g_before+paramDMP.a_g/paramDMP.tau*(paramDMP.g0-paramDMP.g_before)*dt_j;
    
    % r
    paramDMP.r=paramDMP.r_before+paramDMP.a_r/paramDMP.tau*(paramDMP.r0-paramDMP.r_before)*dt_j;
    
    for l=1:paramDMP.L
        w=theta_k(paramDMP.M*(l-1)+1:paramDMP.M*l);
        % f
        f=calc_f(paramDMP,w);   

        % z,y
        paramDMP.z(l)=paramDMP.z_before(l)+(paramDMP.a_z*(paramDMP.b_z*(paramDMP.g-paramDMP.y_before(l))-paramDMP.z_before(l))+f)*dt_j;
        paramDMP.y(l)=paramDMP.y_before(l)+paramDMP.z_before(l)/paramDMP.tau*dt_j;
    end

end

% w: partial of theta_k (policy parameter for each joint)
function f=calc_f(paramDMP,w)
    Psi=zeros(1,paramDMP.M);

    for i=1:paramDMP.M
        h_i=paramDMP.h(i);
        c_i=paramDMP.c(i);
        
        Psi(i)=exp(h_i*cos(paramDMP.phi-c_i)-1);
    end
    
    f=sum(w.*Psi)/sum(Psi)*paramDMP.r;    

end


