clear all;close all;
global vrep clientID yaw_joint_handle_ pitch_joint_handle_

%% set the initial parameter
str_date=datestr(datetime('now'),'mmdd');

% the part of file title (for memo)
name_memo='000';

% the number of experiment
num_experiment=10;

for i_experiment=1:num_experiment
    disp(['experiment:',num2str(i_experiment)])
    
    % set the initial parameter
    [param,constant_,paramRL,result_save]=RL_snake_initial();
    disp(['M:',num2str(paramRL.M),' param_update:',num2str(paramRL.param_update)])

    %% learn
    i_update=1;
    while i_update<=paramRL.num_update

        disp(['update:',num2str(i_update)])

        % adjustment of eigenvalues of Sigma to prevent from being judged non-positive matrix (caused by underflow)
        paramRL.Sigma=re_gen_Sigma(paramRL.Sigma);
        paramRL.theta_test=mvnrnd(paramRL.mu,paramRL.Sigma,paramRL.K);

        % experiment
        [HeadPosition,log_yaw_forces_,log_pitch_forces_]=experiment_vrep(param,paramRL);        
        
        % calculate the cost
        [J,J_input,J_head,sum_J]=calc_J(HeadPosition,log_yaw_forces_,log_pitch_forces_);

        % parameter update (mu,Sigma)
        paramRL.Sigma_before_update=paramRL.Sigma;
        [paramRL]=mu_Sigma_update(J,paramRL);
        
        
        % save the data
        result_save.HeadPosition(i_update,:,:,:)=HeadPosition;
        result_save.sum_J_save(i_update,:)=sum_J;                           % trajectory of sum_i(J_{k,i}) against update
        result_save.mu_save(i_update,:)=paramRL.mu;                         % trajectory of mu against update
        result_save.Sigma_save(i_update,:,:)=paramRL.Sigma;                 % trajectory of Sigma against update
        result_save.theta_test_save(i_update,:,:)=paramRL.theta_test;       % trajectory of theta_k against update
        result_save.lambda_proposed_save(i_update)=paramRL.lambda_proposed; % trajectory of lambda against update (proposed method only)
                        
        % save the data as Mfile
        if paramRL.sw_method==1            
            save (strcat('Proposed',num2str(paramRL.param_update),'_',str_date,'_',num2str(i_experiment),'_',name_memo,'_M',num2str(paramRL.M),'.mat'))
        else
            save (strcat('Existing',num2str(paramRL.param_update),'_',str_date,'_',num2str(i_experiment),'_',name_memo,'_M',num2str(paramRL.M),'.mat'))
        end
        
        i_update=i_update+1;

    end
end


