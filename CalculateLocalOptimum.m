function [solution, metrics] = CalculateLocalOptimum(mvtp, guesses, metrics, rub)
Params = GetModelParams();

solution = [];

tf_guess = Params.tf_max;
default_rlb = -inf * ones(Params.nv, 1);

for i = metrics.iter+1:Params.iter_max
    [v2v_idx, v2o_idx] = GetConstraintsForReducedNLP(mvtp, guesses, default_rlb, rub, 1);
    metrics.n_constraints = [metrics.n_constraints; size(v2v_idx, 1) * 4 + size(v2o_idx, 1) * 2];

    [guesses, tf_guess, opti_flag, iter_time, f_objval] = ...
        SolveTrustRegionOptimalNLP(mvtp, guesses, tf_guess, v2o_idx, v2v_idx, rub);
    
    metrics.iter = i;
    metrics.tf = tf_guess;
    metrics.obj = f_objval;
    metrics.total_time = metrics.total_time + iter_time;
    metrics.opti_flags = [metrics.opti_flags; opti_flag];
    metrics.iter_times = [metrics.iter_times; iter_time];
    
    if ~opti_flag
        return % failed
    else
        collisions = GetOverlappedVehicle(mvtp, guesses(:, :, 1), ...
            guesses(:, :, 2), guesses(:, :, 3));

        if all(collisions == 0)
            solution = guesses;
            return
        end

        rub(collisions) = rub(collisions) + Params.increase_step_ub;
    end
end
end