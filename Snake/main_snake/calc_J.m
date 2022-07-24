function [J,J_input,J_head,sum_J]=calc_J(HeadPosition,log_yaw_forces_,log_pitch_forces_)
    
    for k=1:size(HeadPosition,1)
        for i_t=1:size(HeadPosition,3)
            % cost from the velocity for the target direction
            J_head(k,i_t)=HeadPosition(k,2,i_t);
            
            for l=1:size(log_yaw_forces_,2)
                % cost from the input of yaw joint
                J_input_yaw(l)=log_yaw_forces_(k,l,i_t)^2;
            end
            for l=1:size(log_pitch_forces_,2)
                % cost from the input of pitch joint
                J_input_pitch(l)=log_pitch_forces_(k,l,i_t)^2;
            end
            % cost from the input
            J_input(k,i_t)=0.001*(sum(J_input_yaw)+sum(J_input_pitch));
            
            J(k,i_t)=J_head(k,i_t)+J_input(k,i_t);
                        
        end
    end
    sum_J=sum(J,2);
    
end




