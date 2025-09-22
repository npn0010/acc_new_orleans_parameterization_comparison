function [value,isterminal,direction] = sac_event_function(t,X)

tol = 1e-3;

value = norm([X(2:4)*10; X(5:7)]) - tol;
isterminal = 1;
direction = 0;

end