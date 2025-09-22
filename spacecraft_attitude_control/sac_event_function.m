function [value,isterminal,direction] = sac_event_function(t,x)

tol = 1e-3;

value = norm([x(2:4)*10; x(5:7)]) - tol;
isterminal = 1;
direction = 0;

end