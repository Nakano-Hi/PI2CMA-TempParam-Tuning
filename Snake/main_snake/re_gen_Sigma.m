
% adjustment of eigenvalues of Sigma to prevent from being judged non-positive matrix (caused by underflow)
function Sigma_after=re_gen_Sigma(Sigma_before)
    
    i_reD=0;
    Sigma_after=Sigma_before;

    while rank(Sigma_after)<size(Sigma_after,1)
        [V,D]=eig(Sigma_after);
        
        % Fix to make V,D ``double'' because it sometimes becomes a ``complex double (like a+10^{-17}i)''
        V=real(V);
        D=real(D);
        
        
        threshold_d=i_reD*10^(-5);

        D_re=D;
        for i_D=1:size(D,1)
            d=D(i_D,i_D);
            if d<threshold_d
                D_re(i_D,i_D)=threshold_d;
            end
        end
        
        i_reD=i_reD+1;
        
        Sigma_after=V*D_re*V';
        
    end
    
    % Fix to make Sigma_after ``double'' because it sometimes becomes a ``complex double (like a+10^{-17}i)''
    Sigma_after=real(Sigma_after);
    
end