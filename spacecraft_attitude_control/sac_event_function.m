function [value,isterminal,direction] = sac_event_function(t,X)

tol = 1e-4;

value = norm([X(1) - 1; X(2:7)]) - tol; % The value that we want to be zero
isterminal = 1;  % Halt integration 
direction = 0;   % The zero can be approached from either direction

end