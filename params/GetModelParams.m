function ModelParams = GetModelParams()
ModelParams.x_min = -20;
ModelParams.y_min = -20;
ModelParams.x_max = 20;
ModelParams.y_max = 20;
ModelParams.x_scale = ModelParams.x_max - ModelParams.x_min;
ModelParams.y_scale = ModelParams.y_max - ModelParams.y_min;
ModelParams.xy_resolution = 0.1;
ModelParams.tf_max = 50.0;

ModelParams.trust_radius_min = 3.3;
ModelParams.j_max = 1.5;
ModelParams.K = 4;

ModelParams.solver_options = struct( ...
    'linear_solver', 'mumps', ... % set to ma27 if HSL is acquired
    'max_cpu_time', 1000, ...
    'tol', 1e-3, ...
    'bound_push', 0.01, ...
    'mu_strategy', 'adaptive', ...
    'print_level', 3);

ModelParams.initial_rlb = -4;
ModelParams.initial_rub = 2;
ModelParams.decrease_step = 2;
ModelParams.increase_step_lb = 2;
ModelParams.increase_step_ub = 0.1;
ModelParams.iter_max = 10;

ModelParams.nfe = 100;
ModelParams.nv = 16;
ModelParams.nobs = 8;

end