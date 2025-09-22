clc,clear,close all

%% Example Problems

% prob_name = 'zermelo';
% addpath('zermelo')
% your_func = @(K) zermelo_example(K);
% n = 2;
% lb_lambda = 1e-8;
% ub_lambda = 10;
% num_pso_runs = 10;
% pso_options = optimoptions('particleswarm','UseParallel',false);

prob_name = 'sac';
addpath('spacecraft_attitude_control')
your_func = @(K_p,K_d) sac_example(K_p,K_d);
n = [3 3];
lb_lambda = 1e-8;
ub_lambda = 20;
num_pso_runs = 10;
pso_options = optimoptions('particleswarm','UseParallel',true);

%% Select Control Matrix Parameterization

K_flag = 3; % 1 - diagonal matrix, 2 - givens, 3 - cayley, 4 - cayley mapped

%% Run Particle Swarm Optimization

l = length(n); % number of control matrices

% determine number of variables needed
m = zeros(1,l); % preallocate number of orthogonal matrix parameterization variables for each control matrix
for i = 1:l % loop over number of control matrices
    if K_flag > 1
        m(i) = (n(i)^2-n(i))/2;
    end
end

% set bounds on eigenvalues of control matrices
lb = lb_lambda*ones(sum(n),1);
ub = ub_lambda*ones(sum(n),1);

% set bounds on the orthogonal matrix parameterization variables
if K_flag == 2
    lb = [lb;     zeros(sum(m),1)];
    ub = [ub; 2*pi*ones(sum(m),1)];
elseif K_flag == 3
    lb = [lb; -100*ones(sum(m),1)];
    ub = [ub;  100*ones(sum(m),1)];
elseif K_flag == 4
    lb = [lb; -(pi-0.001)*ones(sum(m),1)];
    ub = [ub;  (pi-0.001)*ones(sum(m),1)];
end

% set other particle swarm options
% pso_options = optimoptions(pso_options,'Display','iter','PlotFcn','pswplotbestf');
pso_options = optimoptions(pso_options,'Display','iter');

% run particle swarm
num_dv    = sum(n)+sum(m);
k_sol     = zeros(num_dv,num_pso_runs);
obj       = zeros(1,num_pso_runs);
output    = cell(1,num_pso_runs);
comp_time = zeros(1,num_pso_runs);
for i = 1:num_pso_runs
    tic
    [k_sol(:,i),obj(i),~,output{i}] = particleswarm(@(k) pso_func(k,your_func,K_flag,l,n,m),num_dv,lb,ub,pso_options);
    comp_time(i) = toc;
end

% determine best solution
[obj_best,obj_best_ind] = min(obj);
k_sol_best = k_sol(:,obj_best_ind);

% re-evaluate to obtain final control matrices
[~,K] = pso_func(k_sol_best,your_func,K_flag,l,n,m);

dt = char(datetime);
dt = strrep(dt,'-','_');
dt = strrep(dt,' ','_');
dt = strrep(dt,':','_');

save([prob_name,'_K_',num2str(K_flag),'_pso_runs_',num2str(num_pso_runs),'_',dt,'.mat'])

%%

% re-evaluate for plotting
[~,out] = your_func(K{:});

