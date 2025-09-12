clc,clear,close all

%% Setup Problem

% addpath('zermelo')
% your_func = @(K) zermelo_example(K);
% control_matrix_sizes = 2;
% lb_lambda = 1e-8;
% ub_lambda = 10;
% num_pso_runs = 1;
% % pso_options = optimoptions('particleswarm',...
% %                            'MaxIterations',10000,...
% %                            'SwarmSize',20,...
% %                            'MaxStallIterations',50,...
% %                            'UseParallel',false);
% pso_options = optimoptions('particleswarm');

addpath('spacecraft_attitude_control')
your_func = @(K_p,K_d) sac_example(K_p,K_d);
% your_func = @(K_p,K_d) sac_example_2(K_p,K_d);
control_matrix_sizes = [3 3];
lb_lambda = 1e-8;
ub_lambda = 10;
num_pso_runs = 1;
pso_options = optimoptions('particleswarm');

%% Control Matrices Settings

% determine whether matrices are diagonal or full
full_flag = 1; % 0 - diagonol matrix, 1 - full matrix

% determine type of parameterization to be used for full matrices
param_flag = 2; % 1 - euler, 2 - cayley

%% Run Particle Swarm Optimization

% number of control matrices
l = length(control_matrix_sizes);

% determine number of variables needed
n = zeros(l,1);
m = zeros(l,1);
for i = 1:l
    n(i) = control_matrix_sizes(i);
    if full_flag
        m(i) = (n(i)^2-n(i))/2;
    end
end

% set bounds on eigenvalues of control matrices
lb = lb_lambda*ones(sum(n),1);
ub = ub_lambda*ones(sum(n),1);

% set bounds on the parameters that parameterize the off-diagonal elements
if full_flag
    if param_flag == 1
        lb = [lb; zeros(sum(m),1)];
        ub = [ub;  ones(sum(m),1)];
    elseif param_flag == 2
        lb = [lb; -100*ones(sum(m),1)];
        ub = [ub;  100*ones(sum(m),1)];
    end
end

% set other particle swarm options
pso_options = optimoptions(pso_options,'Display','iter','PlotFcn','pswplotbestf');

% run particle swarm
k_sol = zeros(sum(n+m),num_pso_runs);
obj   = zeros(1,num_pso_runs);
for i = 1:num_pso_runs
    [k_sol(:,i),obj(i)] = particleswarm(@(k) pso_func(k,your_func,param_flag,l,n,m),sum(n+m),lb,ub,pso_options);
end

% determine best solution
[obj_best,obj_best_ind] = min(obj);
k_sol_best = k_sol(:,obj_best_ind);

% re-evaluate to obtain final control matrices
[~,K] = pso_func(k_sol_best,your_func,param_flag,l,n,m);

%%

% re-evaluate for plotting
[~,out] = your_func(K{:});

