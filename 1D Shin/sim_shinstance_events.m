function [value,isterminal,direction] = sim_shinstance_events(t,x,shin)
% Locate the time when the spring reaches maximum compression; stop
% integration. Also locate the time when the object has return to its
% rest position; stop integration.
%
% The length of the shin is given by
%
%   L(t) = x(4);
%
% When the sprign is compressed to its maximum the length of the shin
% reaches its mininum therefore 

    minCheck = shin.l1min - x(2);

% The shin is at its maximum length when the spring is at its minimum 
% compression of the spring happens when

    maxCheck = shin.l1max - x(2);

% Position difference

% Event to check lift-off 

value = [minCheck;maxCheck];
isterminal = [0;1];
direction = [1;-1];
end