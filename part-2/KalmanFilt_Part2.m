clear; % Clear variables
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(7:9,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)
     if (i == 1)
        oldTime = 0;
    end
    newTime = sampledData(1,i).t;
    dt = double(newTime - oldTime);
    angVel = sampledData(1,i).omg;
    acc = sampledData(1,i).acc;
   [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt);
    oldTime = newTime;
    z_t = Z(:,i);
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);
    uPrev = uCurr;
    covarPrev = covar_curr;
    savedStates(:,i)=uCurr;

end
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);