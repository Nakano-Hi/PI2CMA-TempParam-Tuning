% calculate the target position using DMP
function [target_angle, target_angle_vel,paramDMP] = calc_target(paramDMP,paramRL,k,t_sim,t_sim_old)
    dt=t_sim-t_sim_old;

    target_angle = zeros(1, paramDMP.L);
    target_angle_vel = zeros(1, paramDMP.L);
    
    % paramRL.theta_test :policy parameter
    
    % substitute the result of DMP into target angle
    theta_k=paramRL.theta_test(k,:);
    paramDMP=calc_DMP(paramDMP,theta_k,dt);
    target_angle(1,:) = paramDMP.y; 
end


