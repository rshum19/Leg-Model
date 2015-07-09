function dx = odefun_shinflight_dyn(t, X, shin, params )
%% ODEFUN_FLIGHT_DYN_COM 

%% Read constant parameters
l1max = shin.l1max;
Ksp = shin.spring.Ksp;
Kd = shin.damper.Kd;
g = shin.g;

%% ------------- Forces --------------------
th1 = X(1);
l1 = X(2);
ycm = X(3);
dth1 = X(4);
dl1 = X(5);
dycm = X(6);

%%  Determine if in flight or stance phase

% Input torques
tau1 = 0;

% Ground reaction forces
Fx = 0;
Fy = 0;

% Spring model 
Fsp = 0;

% Damper model
Fd = 0;

gamma = [tau1; Fsp+Fd; Fy];

%% ------------- Equation of Motion ---------
[D,h,G,B] = Eval_ShinFlight_DynFunc(X);
ddx = inv(D)*(B*gamma-h-G);

% Assemble output
dx = [X(4:end);
      ddx];
 
end

