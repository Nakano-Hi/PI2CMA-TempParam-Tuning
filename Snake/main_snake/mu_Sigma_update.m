
function [paramRL]=mu_Sigma_update(J,paramRL)

    % calculate S
    S=calc_S(J);
    
    % calculate P
    if paramRL.sw_method==1
        
        [lambda_proposed]=calc_best_lambda(paramRL,S);
        P=calc_P_constant(S,lambda_proposed);
        paramRL.lambda_proposed=lambda_proposed;
                
    else
        P=calc_P_existing(S,paramRL.param_update);
    end

    % calculate theta_new Sigma_new
    [mu_new,Sigma_new]=calc_theta_Sigma(P,paramRL.theta_test,paramRL.mu);
    
    % update mu and Sigma
    paramRL.mu=mu_new;
    paramRL.Sigma=Sigma_new;
    
    
end

%% calculate cost-to-go (S_ki)
function S=calc_S(J)
    N=size(J,2);
    for k=1:size(J,1)
        for i_t=1:N
            S(k,i_t)=sum(J(k,i_t:N));
        end
    end
end

%% calculate P_ki by constant lambda
function P=calc_P_constant(S,param_update)
    lambda=param_update;
        
    % calculate P in the reverse time direction (to avoid P=NaN in high performance)
    for i_t=size(S,2):-1:1
        c_P=1;
        P_u=exp(-S(:,i_t)/lambda);
        P_b=sum(P_u);
        P(:,i_t)=P_u/P_b;

        while any(any(isnan(P)))
            P_u=exp(-S(:,i_t)/lambda+c_P*0.1);
            P_b=sum(P_u);
            P(:,i_t)=P_u/P_b;
            c_P=c_P+1;
        end
    end
end


%% calculate P_ki by existing method
function P=calc_P_existing(S,param_update)
    h=param_update;
    
    min_S=min(S);
    max_S=max(S);
    for k=1:size(S,1)
        for i_S=1:size(S,2)
            x_S(k,i_S)=(S(k,i_S)-min_S(i_S))/(max_S(i_S)-min_S(i_S));
        end
    end
    
    % calculate P in the reverse time direction (to avoid P=NaN in high performance)
    for i_t=size(x_S,2):-1:1
        P_u=exp(-x_S(:,i_t)*h);
        P_b=sum(P_u);
        P(:,i_t)=P_u/P_b;
    end
end


%% determine lambda for proposed method
function [lambda_best]=calc_best_lambda(paramRL,S)
    % initial setting 
    magnification_lambda=0.98;
    K=size(paramRL.theta_test,1);
    lambda_train=max(paramRL.lambda_proposed*2,100);
    
    % calculate mu_old, Sigma_old
    mu_old=mean(paramRL.theta_test);
    Sigma_old=1/K*(paramRL.theta_test-paramRL.mu)'*(paramRL.theta_test-paramRL.mu);
    
    sw_determine_lambda=0;
    sw_initial=0;
    sw_F=0;
    count_V=0;
    
    V_before=-9999; % tentative value
    F_max=-1; % tentative value
    
    % gradually decrease lambda and find best lambda
    while sw_determine_lambda==0 
    
        [F,ln_V]=calc_FV(S,lambda_train,paramRL,mu_old,Sigma_old);
        disp(['(calculating) lambda:',num2str(lambda_train),',F:',num2str(F)])

        % Increase lambda when F(lambda) for initial lambda is larger than alpha
        if sw_initial==0
            if paramRL.param_update<=F
                lambda_train=lambda_train*2;
                continue
            else
                sw_initial=1;
            end
        end

        if paramRL.param_update<=F % if alpha<=F
            sw_F=1;
            if ln_V>V_before % if V(lambda) is the largest among those already observed
                lambda_best=lambda_train;
                disp(['lambda:',num2str(lambda_best),',F:',num2str(F)])
                V_before=ln_V;
                count_V=0;
            else
                count_V=count_V+1;
                
                if count_V>=3
                    sw_determine_lambda=1;
                end            
            end
        else
            sw_F=0;
            count_V=0;
            if F_max<F
               F_max=F;
               lambda_F=lambda_train;                
            end           
        end
        
        % if lambda is substantially lower than one using previous update,
        % break and choose lambda as one whose F(lambda) is largest
        % among those already observed
        if lambda_train<=paramRL.lambda_proposed/3 && sw_F==0
            lambda_best=lambda_F;
            disp(['lambda:',num2str(lambda_best),',F:',num2str(F)])
            break
        end
        
        lambda_train=lambda_train*magnification_lambda;
        
    end   
    
end


% calculate F,V
function [F_mahal,ln_V_mahal]=calc_FV(S,lambda_train,paramRL,mu_old,Sigma_old)
    % the number of samples for Monte Carlo
    R=10000;

    % calculate P_lambda
    P_lambda=calc_P_constant(S,lambda_train);
    % calculate mu_lambda, Sigma_lambda
    [mu_lambda,Sigma_lambda]=calc_theta_Sigma(P_lambda,paramRL.theta_test,paramRL.mu);
    % generate samples for Monte Carlo
    theta_monte=mvnrnd(mu_lambda,Sigma_lambda,R);
    
    % adjustment of eigenvalues of Sigma to prevent from being judged non-positive matrix (caused by underflow)
    Sigma_lambda=re_gen_Sigma(Sigma_lambda);
    Sigma_old=re_gen_Sigma(Sigma_old);
    
        
    % calculate F
    F_mahal=calc_F_mahal(theta_monte,Sigma_lambda,mu_lambda,Sigma_old,mu_old);    
    
    % calculate log(V)
    logdet_Sigma_lambda=log_det(Sigma_lambda);
    ln_V_mahal=logdet_Sigma_lambda+log(F_mahal);
end




%% calculate mu,Sigma
function [mu_new,Sigma_new]=calc_theta_Sigma(P,theta_test,mu)
    K=size(P,1);
    N=size(P,2);
    
    sum_P=zeros(K,1);
    for k=1:K
        for i_t=1:N
            sum_P(k)=sum_P(k)+P(k,i_t)*(N-i_t)/sum(0:N-1);
        end
    end
    
    mu_new=mu;
    Sigma_new=zeros(size(mu,2),size(mu,2));
    
    for k=1:K
        mu_new=mu_new+sum_P(k)*(theta_test(k,:)-mu);
        Sigma_new=Sigma_new+sum_P(k)*(theta_test(k,:)-mu)'*(theta_test(k,:)-mu);
    end    

end

    
% calculate mahalanobis distance
function mahal_monte=calc_mahal(theta_monte,Sigma_monte,mu_monte)
    mahal_monte=zeros(1,size(theta_monte,1));
    for i_monte=1:size(theta_monte,1)    
        mahal_monte(i_monte)=(theta_monte(i_monte,:)-mu_monte)*inv(Sigma_monte)*(theta_monte(i_monte,:)-mu_monte)';      
    end

end
    
    
% calculate F by Monte Carlo
function F_mahal=calc_F_mahal(theta_monte,Sigma_lambda,mu_lambda,Sigma_old,mu_old)

    mahal_lambda=calc_mahal(theta_monte,Sigma_lambda,mu_lambda);
    mahal_old=calc_mahal(theta_monte,Sigma_old,mu_old);
    
    logdet_sigma_lambda=log_det(Sigma_lambda);
    logdet_sigma_old=log_det(Sigma_old);
    
    F_monte_det=sqrt(exp(logdet_sigma_lambda-logdet_sigma_old));
    
    F_monte_i=zeros(1,size(theta_monte,1));
    for i_monte=1:size(theta_monte,1)    
        F_monte_i(i_monte)=max(1-F_monte_det*exp(-1/2*(mahal_old(i_monte)-mahal_lambda(i_monte))),0);
    end
    F_mahal=mean(F_monte_i);
        
end

% calculate log(|Sigma|)
function logdet_sigma=log_det(Sigma)
    logdet_sigma=2*sum(log(diag(chol(Sigma))));
end

        
    
    
    