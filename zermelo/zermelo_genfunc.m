clc,clear,close all

% time
t  = sym('t');

% states
x = sym('x');
y = sym('y');
X = [x; y];

% control
U = sym('U',[2 1]);

% dynamics
f    = -x*cos(y)*sin(x);
g    = -y*cos(y)*sin(x);
d    = [f; g];
dXdt = U + d;

% gain matrix
K = sym('K',[2 2]);

% lyapunov function
V = 1/2*(X.'*X);

% lyapunov function time derivative
dVdt = jacobian(V,X)*dXdt + jacobian(V,t);

% feedback control law
U_op = -d - (X.'*K).';

U_op_norm = sqrt(U_op.'*U_op);

% substitute control law
dXdt = subs(dXdt,U,U_op);
dVdt = subs(dVdt,U,U_op);

% for objective (J) calculation
J    = sym('J');
dJdt = 1/2*(U_op.'*U_op);

% generate dynamics function file
matlabFunction([dXdt; dJdt],'Vars',{t,[X;J],K},'File','zermelo_eom.m');

% generate functions for plotting
matlabFunction(     U_op,'Vars',{[X;J],K},'File','zermelo_U.m');
matlabFunction(U_op_norm,'Vars',{[X;J],K},'File','zermelo_U_norm.m');
matlabFunction(        V,'Vars',{[X;J],K},'File','zermelo_V.m');
matlabFunction(     dVdt,'Vars',{[X;J],K},'File','zermelo_dVdt.m');
matlabFunction(        f,'Vars',{x,y},'File','zermelo_f.m');
matlabFunction(        g,'Vars',{x,y},'File','zermelo_g.m');