function [results, is_ready, wall_time, obj_val] = SolveTrustRegionGroupNLP(mvtp, idx, guesses, v2o_idx, v2v_idx, rub)
Params = GetModelParams();
VehicleParams = GetVehicleParams();

addpath('casadi');
import casadi.*

profiles = mvtp.profiles;
obstacles = mvtp.obstacles;
nfe = Params.nfe;
radius = VehicleParams.radius;

nvg = length(idx);

tf = Params.tf_max;

id = ones(nvg, nfe);
x = MX.sym('x', nvg, nfe);
b_x = cat(3, Params.x_min * id, Params.x_max * id);

y = MX.sym('y', nvg, nfe);
b_y = cat(3, Params.y_min * id, Params.y_max * id);

v = MX.sym('v', nvg, nfe);
b_v = cat(3, -VehicleParams.v_max * id, VehicleParams.v_max * id);

a = MX.sym('a', nvg, nfe);
b_a = cat(3, -VehicleParams.a_max * id, VehicleParams.a_max * id);

phi = MX.sym('phi', nvg, nfe);
b_phi = cat(3, -VehicleParams.phi_max * id, VehicleParams.phi_max * id);

theta = MX.sym('theta', nvg, nfe);
b_theta = cat(3, -inf * id, inf * id);

omega = MX.sym('omega', nvg, nfe);
b_omega = cat(3, -VehicleParams.omega_max * id, VehicleParams.omega_max * id);

[xf, yf, xr, yr] = GetDiscPositions(x, y, theta);

% parameters
w = 0.1;

f_obj = tf + w * (sumsqr(a) + sumsqr(v .* omega));

g_kin = [];
% kinematic constraints
g_kin = [g_kin, x(:, 2:end) - (x(:, 1:end-1) + tf/nfe * v(:, 1:end-1).*cos(theta(:, 1:end-1)))];
g_kin = [g_kin, y(:, 2:end) - (y(:, 1:end-1) + tf/nfe * v(:, 1:end-1).*sin(theta(:, 1:end-1)))];
g_kin = [g_kin, v(:, 2:end) - (v(:, 1:end-1) + tf/nfe * a(:, 1:end-1))];
g_kin = [g_kin, phi(:, 2:end) - (phi(:, 1:end-1) + tf/nfe * omega(:, 1:end-1))];
g_kin = [g_kin, theta(:, 2:end) - (theta(:, 1:end-1) + tf/nfe * v(:, 1:end-1) .* tan(phi(:, 1:end-1)) / VehicleParams.Lw)];
g_kin = g_kin(:);

% disc constraints
g_v2v = [];
if size(v2v_idx, 1) > 0
    g_v2v = [g_v2v, ...
        (xf(v2v_idx(:, 1)) - xf(v2v_idx(:, 2))) .^ 2 + ...
        (yf(v2v_idx(:, 1)) - yf(v2v_idx(:, 2))) .^ 2 - (2 * radius) ^ 2];
    g_v2v = [g_v2v, ...
        (xf(v2v_idx(:, 1)) - xr(v2v_idx(:, 2))) .^ 2 + ...
        (yf(v2v_idx(:, 1)) - yr(v2v_idx(:, 2))) .^ 2 - (2 * radius) ^ 2];
    g_v2v = [g_v2v, ...
        (xr(v2v_idx(:, 1)) - xr(v2v_idx(:, 2))) .^ 2 + ...
        (yr(v2v_idx(:, 1)) - yr(v2v_idx(:, 2))) .^ 2 - (2 * radius) ^ 2];
    g_v2v = [g_v2v, ...
        (xr(v2v_idx(:, 1)) - xf(v2v_idx(:, 2))) .^ 2 + ...
        (yr(v2v_idx(:, 1)) - yf(v2v_idx(:, 2))) .^ 2 - (2 * radius) ^ 2];
    g_v2v = g_v2v(:);
end

g_v2o = [];
if size(v2o_idx, 1) > 0
    xfi = xf(v2o_idx(:, 1));
    yfi = yf(v2o_idx(:, 1));
    xri = xr(v2o_idx(:, 1));
    yri = yr(v2o_idx(:, 1));
    g_v2o = [g_v2o, ...
        (xfi(:) - obstacles(v2o_idx(:, 2), 1)) .^ 2 + ...
        (yfi(:) - obstacles(v2o_idx(:, 2), 2)) .^ 2 - (radius + obstacles(v2o_idx(:, 2), 3)) .^ 2];
    g_v2o = [g_v2o, ...
        (xri(:) - obstacles(v2o_idx(:, 2), 1)) .^ 2 + ...
        (yri(:) - obstacles(v2o_idx(:, 2), 2)) .^ 2 - (radius + obstacles(v2o_idx(:, 2), 3)) .^ 2];
    g_v2o = g_v2o(:);
end

[xf0, yf0, xr0, yr0] = GetDiscPositions(guesses(idx, :, 1), guesses(idx, :, 2), guesses(idx, :, 3));


% g_tr = [];
% lb_tr = [];
% ub_tr = [];
g_tr = [xf, yf, xr, yr];
lb_xf = xf0; ub_xf = xf0;
lb_yf = yf0; ub_yf = yf0;
lb_xr = xr0; ub_xr = xr0;
lb_yr = yr0; ub_yr = yr0;

for ii = 1:nvg
    trust_radius = max(rub(ii), Params.trust_radius_min);
    lb_xf(ii, :) = xf0(ii, :) - trust_radius; ub_xf(ii, :) = xf0(ii, :) + trust_radius;
    lb_yf(ii, :) = yf0(ii, :) - trust_radius; ub_yf(ii, :) = yf0(ii, :) + trust_radius;
    lb_xr(ii, :) = xr0(ii, :) - trust_radius; ub_xr(ii, :) = xr0(ii, :) + trust_radius;
    lb_yr(ii, :) = yr0(ii, :) - trust_radius; ub_yr(ii, :) = yr0(ii, :) + trust_radius;        
end

lb_tr = [lb_xf lb_yf lb_xr lb_yr];
ub_tr = [ub_xf ub_yf ub_xr ub_yr];


% boundary constraints
b_v(:, [1 end], :) = 0.0;
b_a(:, [1 end], :) = 0.0;
b_phi(:, [1 end], :) = 0.0;
b_omega(:, [1 end], :) = 0.0;

g_bctheta = [sin(theta(:, end)) - sin(profiles(idx, 6)) cos(theta(:, end)) - cos(profiles(idx, 6))];
g_bctheta = g_bctheta(:);

for ii = 1:nvg
    b_x(ii, 1, :) = profiles(idx(ii), 1);
    b_y(ii, 1, :) = profiles(idx(ii), 2);
    b_theta(ii, 1, :) = profiles(idx(ii), 3);

    b_x(ii, end, :) = profiles(idx(ii), 4);
    b_y(ii, end, :) = profiles(idx(ii), 5);
end

guesses_filtered = guesses(idx, :, :);
x0 = guesses_filtered(:);
opti_lbx = cat(3, b_x(:, :, 1), b_y(:, :, 1), b_theta(:, :, 1), b_v(:, :, 1), b_phi(:, :, 1), b_a(:, :, 1), b_omega(:, :, 1));
lbx = opti_lbx(:);
opti_ubx = cat(3, b_x(:, :, 2), b_y(:, :, 2), b_theta(:, :, 2), b_v(:, :, 2), b_phi(:, :, 2), b_a(:, :, 2), b_omega(:, :, 2));
ubx = opti_ubx(:);
 
opti_g = [g_kin; g_bctheta; g_tr(:); g_v2v; g_v2o];
lbg = [zeros(size(g_kin)); zeros(size(g_bctheta)); lb_tr(:); zeros(size(g_v2v)); zeros(size(g_v2o))];
ubg = [zeros(size(g_kin)); zeros(size(g_bctheta)); ub_tr(:); 1e20 * ones(size(g_v2v)); 1e20 * ones(size(g_v2o))];
 
opti_x = [x, y, theta, v, phi, a, omega];
opti_x = opti_x(:);

nlp = struct('x', opti_x, 'f', f_obj, 'g', opti_g);
S = nlpsol('S', 'ipopt', nlp, struct('ipopt', Params.solver_options));
r = S('x0',x0,'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg);

x_opt = full(r.x);
results = reshape(x_opt, [nvg, nfe, 7]);
is_ready = S.stats().success;
wall_time = S.stats().t_wall_total;
obj_val = full(r.f);
end