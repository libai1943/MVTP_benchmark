function [v2v_idx, v2o_idx] = GetConstraintsForReducedNLP(mvtp, guesses, rlb, rub, step)
Params = GetModelParams();
v2v_idx = zeros(0, 2);
v2o_idx = zeros(0, 2);

team_size = size(guesses, [1 2]);
idx = floor(linspace(1, Params.nfe, ceil(Params.nfe / step)));

for ii = 1:Params.nv
    for j = ii+1:Params.nv
        collisions = zeros(Params.nfe, 2);
        
        for k = 2:Params.nfe-1
            ego_pose = guesses(ii, k, [1 2 3]);
            opposite_pose = guesses(j, k, [1 2 3]);
            [~, is_distant1] = IsConfig1FarFromConfig2(ego_pose, opposite_pose, rlb(ii), rub(ii));
            [~, is_distant2] = IsConfig1FarFromConfig2(ego_pose, opposite_pose, rlb(j), rub(j));
            
            if ~(is_distant1 && is_distant2)
                collisions(k, :) = [sub2ind(team_size, ii, k) sub2ind(team_size, j, k)];
            end
        end
        
        included = collisions(idx, :);
        v2v_idx = [v2v_idx; included(included(:, 1) > 0, :)];
    end
        
    for j = 1:Params.nobs
        collisions = zeros(Params.nfe, 2);

        for k = 2:Params.nfe-1
            ego_pose = guesses(ii, k, [1 2 3]);
            ob = mvtp.obstacles(j, :);
            [~, is_distant] = IsConfigFarFromObstacle(ego_pose, ob(1), ob(2), ob(3), rlb(ii), rub(ii));
            
            if ~is_distant
                collisions(k, :) = [sub2ind(team_size, ii, k) j];
            end
        end
        
        included = collisions(idx, :);
        v2o_idx = [v2o_idx; included(included(:, 1) > 0, :)];
    end
end
end


function [distance, is_distant] = IsConfig1FarFromConfig2(vec1, vec2, rlb, rub)
[xf, yf, xr, yr] = GetDiscPositions([vec1(1), vec2(1)], [vec1(2), vec2(2)], [vec1(3), vec2(3)]);
P11 = [xr(1), yr(1)];
P12 = [xf(1), yf(1)];
P21 = [xr(2), yr(2)];
P22 = [xf(2), yf(2)];
distance = min([norm(P11 - P21), norm(P11 - P22), norm(P12 - P21), norm(P12 - P22)]) - 2 * GetVehicleParams().radius;

if ((distance > rub)||(distance < rlb))
    is_distant = 1;
else
    is_distant = 0;
end
end
 
function [distance, is_distant] = IsConfigFarFromObstacle(vec, obs_x, obs_y, obs_radius, rlb, rub)
[xf, yf, xr, yr] = GetDiscPositions(vec(1), vec(2), vec(3));
P1 = [xr, yr];
P2 = [xf, yf];
P0 = [obs_x, obs_y];
distance = min([norm(P1 - P0), norm(P2 - P0)]) - GetVehicleParams().radius - abs(obs_radius);
if ((distance > rub)||(distance < rlb))
    is_distant = 1;
else
    is_distant = 0;
end
end
