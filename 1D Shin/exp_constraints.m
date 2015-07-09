function [ineq_violations, eq_violations ] = exp_constraints(q, shin )

% Last Updated: 7/6/2015

%% Read input

% Query values q
Kd_q = q(1);

% Do forward dynamics

%% Implement constraints subject to:

temp_ceq = [];
temp_c = [];

%% Equality constraints

%% Inequality constraints
idx = 0;

% s.t. Damper coefficient limits
idx = idx + 1;
temp_c(idx) = Kd_q - shin.damper.Kdmax;

idx = idx + 1;
temp_c(idx) = shin.damper.Kdmin - Kd_q;

% Set violations
eq_violations = temp_ceq';
ineq_violations = temp_c';

end

