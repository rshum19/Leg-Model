function [ score ] = exp_criterion(q, shin)


% Query values q
Kd_q = q(1);

% Do forward dynamics
[max_bounce_height] = sim_drop(shin,Kd_q);
  
%% Cost function
score = 0;      % Reset score

% Penalize for not taking off

% Penalize for bounce height
score = max_bounce_height^2;
end

