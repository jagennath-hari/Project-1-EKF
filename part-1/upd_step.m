function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively
%%
C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0]; %C matrix
R = diag([0.00001 0.00001 0.00001 0.00001 0.00001 0.00001]); %R matrix found by tuning

Kt = (covarEst * transpose(C))*pinv((((C * covarEst * transpose(C)) + R))); %Kalman gain 

%%

uCurr = double(uEst + (Kt * (z_t - (C * uEst)))); %mean current
covar_curr = double(covarEst - (Kt * C * covarEst)); %covariance current 
end