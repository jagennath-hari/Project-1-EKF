function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%covarPrev and uPrev are the previous mean and covariance respectively
%angVel is the angular velocity
%acc is the acceleration
%dt is the sampling time
%%
%{
I found everything in syms. The steps are given below. After finding in
syms I have preloaded the matrix to increase computational time. The code is given below.

syms x1 x2 x3 x4 x5 X %state matrix
syms posx posy posz %position
syms q_x q_y q_z %orientation
syms pdotx pdoty pdotz %linear velocity
syms bgx bgy bgz %gyro bias
syms bax bay baz %acc bias
syms wm ng am na %nbg nba Xdot 
syms wmx wmy wmz ngx ngy ngz amx amy amz nax nay naz nbgx nbgy nbgz nbax
nbay nbaz % syms for wm ng am nbg nba na ng 


x1 = [posx; posy; posz]; %positions
x2 = [q_x; q_y; q_z]; %orientations
x3 = [pdotx; pdoty; pdotz]; %linear velocity
x4 = [bgx; bgy; bgz]; %gyro bias
x5 = [bax; bay; baz]; %accelerometer bias

X = [x1; x2; x3; x4; x5]; %state


R = simplify(rotz(q_z)*roty(q_y)*rotx(q_x)); %R matrix 


G = [cos(q_y)*cos(q_z) -sin(q_z) 0;
    cos(q_y)*sin(q_z) cos(q_z) 0;
    -sin(q_y) 0 1];  % G matrix


G = simplify(inv(G)*R); %updating G matrix by post mulitplying with R


wm = [wmx; wmy; wmz]; %wm matrix 
ng = [ngx; ngy; ngz]; %ng matrix
am = [amx; amy; amz]; %am matrix
na = [nax; nay; naz]; %na matrix
nbg = [nbgx; nbgy; nbgz]; %nbg matrix
nba = [nbax; nbay; nbaz]; %nba matrix


Xdot = simplify([x3; G*(wm - x4 - ng); [0; 0; -9.81] +
(R)*(am - x5 - na); nbg; nba]); % Xdot is the predidiction model



syms At %At matrix for predition step

for i=1:length(Xdot)
    At(1:length(Xdot),i) = diff(Xdot, X(i,1));
end %partial differentation for finding At

At = simplify(At) %this will be used to find Ft


syms Ut %Ut matrix for noise
temp = vertcat(ng, na, nbg, nba); %appending all the noise matrix
for i=1:length(temp)
    Ut(1:length(Xdot),i) = diff(Xdot, temp(i,1));
end %partial differentation in terms of noise

Vt = simplify(Ut); %Vt matrix

%}




%%
posx = double(uPrev(1,1));
posy = double(uPrev(2,1));
posz = double(uPrev(3,1));
q_x = double(uPrev(4,1));
q_y = double(uPrev(5,1));
q_z = double(uPrev(6,1));
pdotx = double(uPrev(7,1));
pdoty = double(uPrev(8,1));
pdotz = double(uPrev(9,1));
wmx = double(angVel(1,1));
wmy = double(angVel(2,1));
wmz = double(angVel(3,1));
amx = double(acc(1,1));
amy = double(acc(2,1));
amz = double(acc(3,1));

%%

R = [cos(q_y)*cos(q_z), cos(q_z)*sin(q_x)*sin(q_y) - cos(q_x)*sin(q_z), sin(q_x)*sin(q_z) + cos(q_x)*cos(q_z)*sin(q_y);
    cos(q_y)*sin(q_z), cos(q_x)*cos(q_z) + sin(q_x)*sin(q_y)*sin(q_z), cos(q_x)*sin(q_y)*sin(q_z) - cos(q_z)*sin(q_x);
    -sin(q_y), cos(q_y)*sin(q_x), cos(q_x)*cos(q_y)];
G = pinv([cos(q_y)*cos(q_z), -sin(q_z), 0;
    cos(q_y)*sin(q_z), cos(q_z), 0;
    -sin(q_y), 0, 1])* R;


%%

ngx = 0.001;
ngy = 0.001;
ngz = 0.001;
nax = 0.001;
nay = 0.001;
naz = 0.001;
bgx = 0.001;
bgy = 0.001;
bgz = 0.001;
bax = 0.001;
bay = 0.001;
baz = 0.001; %found by tuning

ng = [ngx; ngy; ngz];
na = [nax; nay; naz];
nbg = [bgx; bgy; bgz];
nba = [bax; bay; baz];
%%
x3 = [uPrev(7,1); uPrev(8,1); uPrev(9,1)];
x4 = [uPrev(10,1); uPrev(11,1); uPrev(12,1)];
x5 = [uPrev(13,1); uPrev(14,1); uPrev(15,1)];  
wm = [angVel(1,1); angVel(2,1); angVel(3,1)];
am = [acc(1,1); acc(2,1); acc(3,1)];

Xdot = [x3; (G*(wm - x4 -ng)); [0; 0; -9.81]+(R*(am-x5-na)); nbg; nba];

%%
posx = double(uPrev(1,1));
posy = double(uPrev(2,1));
posz = double(uPrev(3,1));
q_x = double(uPrev(4,1));
q_y = double(uPrev(5,1));
q_z = double(uPrev(6,1));
pdotx = double(uPrev(7,1));
pdoty = double(uPrev(8,1));
pdotz = double(uPrev(9,1));
wmx = double(angVel(1,1));
wmy = double(angVel(2,1));
wmz = double(angVel(3,1));
amx = double(acc(1,1));
amy = double(acc(2,1));
amz = double(acc(3,1));

%%
At =[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, cos(q_z)*sin(q_y)*(bgx + ngx - wmx), cos(q_z)*(bgy + ngy - wmy) + cos(q_y)*sin(q_z)*(bgx + ngx - wmx), 0, 0, 0, -cos(q_y)*cos(q_z), sin(q_z), 0, 0, 0, 0;
    0, 0, 0, 0, sin(q_y)*sin(q_z)*(bgx + ngx - wmx), sin(q_z)*(bgy + ngy - wmy) - cos(q_y)*cos(q_z)*(bgx + ngx - wmx), 0, 0, 0, -cos(q_y)*sin(q_z), -cos(q_z), 0, 0, 0, 0;
    0, 0, 0, 0, cos(q_y)*(bgx + ngx - wmx), 0, 0, 0, 0, sin(q_y), 0, -1, 0, 0, 0;
    0, 0, 0, - (sin(q_x)*sin(q_z) + cos(q_x)*cos(q_z)*sin(q_y))*(bay - amy + nay) - (cos(q_x)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y))*(baz - amz + naz), cos(q_z)*sin(q_y)*(bax - amx + nax) - cos(q_x)*cos(q_y)*cos(q_z)*(baz - amz + naz) - cos(q_y)*cos(q_z)*sin(q_x)*(bay - amy + nay), (cos(q_x)*cos(q_z) + sin(q_x)*sin(q_y)*sin(q_z))*(bay - amy + nay) - (cos(q_z)*sin(q_x) - cos(q_x)*sin(q_y)*sin(q_z))*(baz - amz + naz) + cos(q_y)*sin(q_z)*(bax - amx + nax), 0, 0, 0, 0, 0, 0, -cos(q_y)*cos(q_z), (cos(q_x)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y)), - sin(q_x)*sin(q_z) - cos(q_x)*cos(q_z)*sin(q_y);
    0, 0, 0, (cos(q_z)*sin(q_x) - cos(q_x)*sin(q_y)*sin(q_z))*(bay - amy + nay) + (cos(q_x)*cos(q_z) + sin(q_x)*sin(q_y)*sin(q_z))*(baz - amz + naz), sin(q_y)*sin(q_z)*(bax - amx + nax) - cos(q_x)*cos(q_y)*sin(q_z)*(baz - amz + naz) - cos(q_y)*sin(q_x)*sin(q_z)*(bay - amy + nay), (cos(q_x)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y))*(bay - amy + nay) - (sin(q_x)*sin(q_z) + cos(q_x)*cos(q_z)*sin(q_y))*(baz - amz + naz) - cos(q_y)*cos(q_z)*(bax - amx + nax), 0, 0, 0, 0, 0, 0, -cos(q_y)*sin(q_z), - cos(q_x)*cos(q_z) - sin(q_x)*sin(q_y)*sin(q_z), (cos(q_z)*sin(q_x) - cos(q_x)*sin(q_y)*sin(q_z));
    0, 0, 0, cos(q_y)*sin(q_x)*(baz - amz + naz) - cos(q_x)*cos(q_y)*(bay - amy + nay), cos(q_y)*(bax - amx + nax) + cos(q_x)*sin(q_y)*(baz - amz + naz) + sin(q_x)*sin(q_y)*(bay - amy + nay), 0, 0, 0, 0, 0, 0, 0, sin(q_y), -cos(q_y)*sin(q_x), -cos(q_x)*cos(q_y);
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];


Ft = eye(15) + dt*At; %Ft for prediction step

%%

Vt = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    -cos(q_y)*cos(q_z), sin(q_z), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    -cos(q_y)*sin(q_z), -cos(q_z), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    sin(q_y), 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, -cos(q_y)*cos(q_z), cos(q_x)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y), - sin(q_x)*sin(q_z) - cos(q_x)*cos(q_z)*sin(q_y), 0, 0, 0, 0, 0, 0;
    0, 0, 0, -cos(q_y)*sin(q_z), - cos(q_x)*cos(q_z) - sin(q_x)*sin(q_y)*sin(q_z), cos(q_z)*sin(q_x) - cos(q_x)*sin(q_y)*sin(q_z), 0, 0, 0, 0, 0, 0;
    0, 0, 0, sin(q_y), -cos(q_y)*sin(q_x), -cos(q_x)*cos(q_y), 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];


Qd = diag([ngx ngy ngz nax nay naz bgx bgy bgz bax bay baz])*dt; %Qd matrix found by tuning the values

%%

uEst = double(uPrev + Xdot*dt); %Estimated mean
covarEst = double(Ft*covarPrev*transpose(Ft)+ Vt*Qd*transpose(Vt)); %Estimated Covariance

end
 
