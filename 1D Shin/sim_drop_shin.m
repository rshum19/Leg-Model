%% SIM_DROP_SHIN simulates the drop the shin

%% --------------------- Initialize Workspace -----------------------
clear ; close all; clc;

% initialize shin
init_shin;

% set Ground properties
params.ground.Kg = 10e4;     % [N/m]
params.ground.Bg = 75;       % [Ns/m]
params.ground.y_td = 0;
params.vardamping = 0;

% Set initial conditions
th1_0 = 0;
l1_0 = 0.3;
ycm_0 = 2;

dth1_0 = 0;
dl1_0 = 0;
dycm_0 = 0;

X0 = [ th1_0; l1_0; ycm_0;...
       dth1_0; dl1_0; dycm_0];

% Intiialize variables to store simulation results
tout = 0;
Xout = X0.';

%% ------------------ Flight Phase ------------------------------
fprintf('Flight\n')
options = odeset('RelTol',1e-2,'AbsTol',1e-2,...
                     'Events',@(t,x)sim_shinflight_events(t,x,shin),'Stats','off');

tstart = 0;
tend = 10;

[t,X] = ode45(@(t,x)odefun_shinflight_dyn(t,x,shin, params),[tstart,tend],X0,options);

% Concatenate output
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];
%% ------------- Flight --> Stance Phase -----------------------
% This defines the initial conditions for the simulation of the leg during contact with ground

% Change in state variable vector reduces in size from:
%   q = [th1, l1, ycm, dth1, dl1, dycm];
% to
%   q = [th1, l1, dth1, dl1];
%
% Extract state just before impact w/ ground (qf-)
fprintf('Flight --> Stance\n')
qf = X(end,1:3);
dqf = X(end,4:6);
Q_fminus = [qf, dqf];

[D,~,~,~] = Eval_ShinFlight_DynFunc(Q_fminus);
A = D(1:2,1:2);

mt = shin.m1 + shin.m2;
Pf2com = COMrel2Foot(shin, Q_fminus);
dPf2comdq = Pf2com_jacobian(shin, Q_fminus);
dqs = inv(A + mt*dPf2comdq'*dPf2comdq)*[A, mt*dPf2comdq']*dqf';

Q_stnc_plus = [qf(1:2),dqs'];

%% ------------------ Stance Phase ------------------------------
X0 = Q_stnc_plus;
tstart = tout(end);

options = odeset('RelTol',1e-2,'AbsTol',1e-2,...
                     'Events', @(t,x)sim_shinstance_events(t,x,shin), 'Stats','off');
                     
[t,X] = ode45(@(t,x)odefun_shinstance_dyn(t,x,shin, params),[tstart,tend],X0,options);

% need to get CoM position given joint angles and foot location
for i = 1:size(X,1)
    Xcm(i,:) = COMrel2Foot(shin, X(i,:));
end

% estimate CoM velocity
for i = 2:size(X,1)
    dXcm(i,:) = (Xcm(i)-Xcm(i-1))/(t(i)-t(i-1));
end

% Concatenate output
nt = length(t);
tout = [tout; t(2:nt)];
Xtemp = [X(2:nt,1:2), Xcm(2:nt),X(2:nt,3:4),dXcm(2:nt)];
Xout = [Xout; Xtemp];

%% ---------------- Stance --> Flight ----------------------------

dy_lom = Xout(end,6);
dy_lop = shin.m2/(shin.m1 + shin.m2)*dy_lom;

% find position difference
for i = 2:size(Xout,1)
    pos_diff(i) = Xout(i,3)-Xout(i-1,3);
end


figure
% COM Position plot
subplot(2,1,1)
plot(tout,Xout(:,3))
ylabel('COM y-position')

% COM Velocity plot
subplot(2,1,2)
plot(tout,Xout(:,6)) 
hold on;
plot([tout(1),tout(end)],zeros(1,2),'k-')
hold off;
ylabel('COM y-velocity')
xlabel('Time (sec)')

%% ----------------- Flight --------------------------------------
options = odeset('RelTol',1e-2,'AbsTol',1e-2,'Events',@(t,x)sim_shinflight_events(t,x,shin));
tstart = tout(end);
X0 = [ th1_0; l1_0; Xout(end,3);...
       dth1_0; dl1_0; dy_lop];
   
[t,X] = ode45(@(t,x)odefun_shinflight_dyn(t,x,shin, params),[tstart,tend],X0,options);

% Find bounce height
max_bounce_height = max(X(3,:));

% Concatenate results
nt = length(t);
tout = [tout; t(2:nt)];
Xout = [Xout; X(2:nt,:)];


% Calculate Ground Reaction Force
% Ground Reaction Force
lspring = shin.l1max - Xout(:,2);

[GRF, Fs, Fd] = calc_GRF(lspring,Xout(:,5),shin.spring.Ksp,shin.damper.Kd);


figure
subplot(3,1,1)
plot(tout, GRF);
ylabel('GRF')

subplot(3,1,2)
plot(tout, Fs);
ylabel('Fs')

subplot(3,1,3)
plot(tout, Fd);
ylabel('Fd')
xlabel('time')

%% ------------------ Animation & Plots -------------------------
%shin_animation(shin, Xout)
%shin_animation(shin, Xi)


figure
% COM Position plot
subplot(2,2,1)
plot(tout,Xout(:,3))
ylabel('COM y-position')

% COM Velocity plot
subplot(2,2,3)
plot(tout,Xout(:,6)) 
hold on;
plot([tout(1),tout(end)],zeros(1,2),'k-')
hold off;
ylabel('COM y-velocity')
xlabel('Time (sec)')

% Spring  Position
subplot(2,2,2)
plot(tout,Xout(:,2))

subplot(2,2,4)
plot(tout,Xout(:,5))

