function [solution, metrics, rub] = IdentifyHomotopyClass(mvtp)
Params = GetModelParams();

tic
guesses = GenerateInitialGuess(mvtp);
metrics.ha_time = toc;

solution = [];

metrics.total_time = 0.0;

metrics.n_constraints = [];
metrics.opti_flags = [];
metrics.iter_times = [];
metrics.partitions = [];


rlb = Params.initial_rlb * ones(Params.nv, 1);
rub = Params.initial_rub * ones(Params.nv, 1);


for ii = 1:Params.iter_max
    [v2v_idx, v2o_idx] = GetConstraintsForReducedNLP(mvtp, guesses, rlb, rub, Params.K);

    metrics.iter = ii;
    metrics.n_constraints = [metrics.n_constraints; size(v2v_idx, 1) * 4 + size(v2o_idx, 1) * 2];

    partitions = VehiclePartition(v2v_idx);
    [v2v_parts, v2o_parts] = CollisionConstraintsPartition(partitions, v2v_idx, v2o_idx);

    partition_num = size(partitions, 1);
    metrics.partitions = [metrics.partitions; partition_num];
    metrics.obj = 0.0;

    iter_time = 0.0;
    opti_flags = false(Params.nv, 1);

    for jj = 1:partition_num
        part = partitions{jj};

        [result, opti_flag, time, f_obj] = SolveTrustRegionGroupNLP(mvtp, ...
            part, guesses, v2o_parts{jj}, v2v_parts{jj}, rub(part));
        iter_time = max(iter_time, time); % ideal parallel execution
        metrics.obj = metrics.obj + f_obj;

        if opti_flag
            guesses(part, :, :) = result;
            opti_flags(part) = true;
        else
            rlb(part) = rlb(part) + Params.decrease_step;
        end
    end
    
    metrics.total_time = metrics.total_time + iter_time;
    metrics.iter_times = [metrics.iter_times; iter_time];
    metrics.opti_flags = [metrics.opti_flags; all(opti_flags)];

    [collisions, errors] = GetOverlappedVehicle(mvtp, guesses(:, :, 1), ...
        guesses(:, :, 2), guesses(:, :, 3));

    alter = opti_flags & collisions;
    rlb(alter) = max(rlb(alter) - Params.increase_step_lb, Params.initial_rlb);
    rub(alter) = rub(alter) + Params.increase_step_ub;

    if all(opti_flags)
        J = min(errors(:));
        fprintf('J=%f\n', J);

        if J + Params.j_max > 0
            solution = guesses;
            return
        end
    end
   
end
end

function [v2v_parts, v2o_parts] = CollisionConstraintsPartition(partitions, v2v_idx, v2o_idx)
Params = GetModelParams();

partition_num = size(partitions, 1);
state_size = [Params.nv, Params.nfe];
v2v_parts = cell(partition_num, 1);
v2o_parts = cell(partition_num, 1);

for ii = 1:partition_num
    part = partitions{ii};
    part_size = [length(part) Params.nfe];
    v2v_filtered = [];
    v2o_filtered = [];
    for jj = 1:size(v2v_idx, 1)
        [vi_r, vi_k] = ind2sub(state_size, v2v_idx(jj, 1));
        [vj_r, ~] = ind2sub(state_size, v2v_idx(jj, 2));

        vir_idx = find(part == vi_r);
        vjr_idx = find(part == vj_r);

        if ~isempty(vir_idx) && ~isempty(vjr_idx)
            v2v_filtered = [v2v_filtered; sub2ind(part_size, vir_idx, vi_k), sub2ind(part_size, vjr_idx, vi_k)];
        end
    end
    v2v_parts{ii} = v2v_filtered;

    for jj = 1:size(v2o_idx, 1)
        [vi_r, vi_k] = ind2sub(state_size, v2o_idx(jj, 1));
        vir_idx = find(part == vi_r);

        if ~isempty(vir_idx)
            v2o_filtered = [v2o_filtered; sub2ind(part_size, vir_idx, vi_k), v2o_idx(jj, 2)];
        end
    end
    v2o_parts{ii} = v2o_filtered;
end

end

function [partitions] = VehiclePartition(v2v_idx)
Params = GetModelParams();

state_size = [Params.nv, Params.nfe];

if isempty(v2v_idx)
    partitions = num2cell(1:Params.nv)';
    return;
end

G = graph;
G = addnode(G, Params.nv);

for vk = 1:size(v2v_idx, 1)
    [vi_r, ~] = ind2sub(state_size, v2v_idx(vk, 1));
    [vj_r, ~] = ind2sub(state_size, v2v_idx(vk, 2));
    G = addedge(G, vi_r, vj_r);
end

subgraph_idx = conncomp(G);
subgraph_count = max(subgraph_idx);
partitions = cell(subgraph_count, 1);

for ii = 1:subgraph_count
    partitions{ii} = find(subgraph_idx == ii);
end

end