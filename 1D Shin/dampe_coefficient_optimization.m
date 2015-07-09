% Optimization of MR Damper damping coefficient for compression and
% expansion phases.
%
% Need to do optimizations

init_shin;
p0 = 1e2;
%sim_drop( shin, p0);

fprintf('Perfoming optimization...');
[p_opt,fval,exitflag,output] = fmincon(@(q)exp_criterion(q,shin),p0,[],[],[],[],[],[],@(q)exp_constraints(q,shin));
fprintf('Done\n');