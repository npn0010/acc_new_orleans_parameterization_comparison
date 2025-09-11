function [value,isterminal,direction] = zermelo_event_function(t,X)
  value = norm(X(1:2)) - 1e-3; % The value that we want to be zero
  isterminal = 1;  % Halt integration 
  direction = 0;   % The zero can be approached from either direction
end