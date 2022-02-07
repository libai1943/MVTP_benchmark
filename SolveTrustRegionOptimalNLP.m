function [results, tf_result, is_ready, wall_time, f_objval] = SolveTrustRegionOptimalNLP(mvtp, guesses, tf_guess, v2o_idx, v2v_idx, rub)
Params = GetModelParams();
VehicleParams = GetVehicleParams();

addpath('casadi');
import casadi.*

profiles = mvtp.profiles;
obstacles = mvtp.obstacles;
nv = Params.nv;
nfe = Params.nfe;
radius = VehicleParams.radius;

tf = MX.sym('tf');
b_tf = [0.1, Params.tf_max];

x = MX.sym('x', nv, nfe);
y = MX.sym('y', nv, nfe);
theta = MX.sym('theta', nv, nfe);
b_x = cat(3, Params.x_min * ones(nv, nfe), Params.x_max * ones(nv, nfe));
b_y = cat(3, Params.y_min * ones(nv, nfe), Params.y_max * ones(nv, nfe));
b_theta = cat(3, -inf * ones(nv, nfe), inf * ones(nv, nfe));

v = MX.sym('v', nv, nfe);
b_v = cat(3, -VehicleParams.v_max * ones(nv, nfe), VehicleParams.v_max * ones(nv, nfe));

phi = MX.sym('phi', nv, nfe);
b_phi = cat(3, -VehicleParams.phi_max * ones(nv, nfe), VehicleParams.phi_max * ones(nv, nfe));

a = MX.sym('a', nv, nfe);
b_a = cat(3, -VehicleParams.a_max * ones(nv, nfe), VehicleParams.a_max * ones(nv, nfe));

omega = MX.sym('omega', nv, nfe);
b_omega = cat(3, -VehicleParams.omega_max * ones(nv, nfe), VehicleParams.omega_max * ones(nv, nfe));

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

[xf0, yf0, xr0, yr0] = GetDiscPositions(guesses(:, :, 1), guesses(:, :, 2), guesses(:, :, 3));

g_tr = [xf, yf, xr, yr];
lb_xf = xf0; ub_xf = xf0;
lb_yf = yf0; ub_yf = yf0;
lb_xr = xr0; ub_xr = xr0;
lb_yr = yr0; ub_yr = yr0;

for ii = 1:nv
    trust_radius = max(rub(ii), Params.trust_radius_min);
    lb_xf(ii, :) = lb_xf(ii, :) - trust_radius; ub_xf(ii, :) = ub_xf(ii, :) + trust_radius;
    lb_yf(ii, :) = lb_yf(ii, :) - trust_radius; ub_yf(ii, :) = ub_yf(ii, :) + trust_radius;
    lb_xr(ii, :) = lb_xr(ii, :) - trust_radius; ub_xr(ii, :) = ub_xr(ii, :) + trust_radius;
    lb_yr(ii, :) = lb_yr(ii, :) - trust_radius; ub_yr(ii, :) = ub_yr(ii, :) + trust_radius;        
end

lb_tr = [lb_xf lb_yf lb_xr lb_yr];
ub_tr = [ub_xf ub_yf ub_xr ub_yr];


% boundary constraints
b_v(:, [1 end], :) = 0.0;
b_a(:, [1 end], :) = 0.0;
b_phi(:, [1 end], :) = 0.0;
b_omega(:, [1 end], :) = 0.0;

g_bctheta = [sin(theta(:, end)) - sin(profiles(:, 6)) cos(theta(:, end)) - cos(profiles(:, 6))];
g_bctheta = g_bctheta(:);

for ii = 1:nv
    b_x(ii, 1, :) = profiles(ii, 1);
    b_y(ii, 1, :) = profiles(ii, 2);
    b_theta(ii, 1, :) = profiles(ii, 3);

    b_x(ii, end, :) = profiles(ii, 4);
    b_y(ii, end, :) = profiles(ii, 5);
end

guesses0 = [guesses(:, :, 1) guesses(:, :, 2) guesses(:, :, 3) guesses(:, :, 4) guesses(:, :, 5) guesses(:, :, 6) guesses(:, :, 7)];
x0 = [tf_guess; guesses0(:)];
opti_lbx = cat(3, b_x(:, :, 1), b_y(:, :, 1), b_theta(:, :, 1), b_v(:, :, 1), b_phi(:, :, 1), b_a(:, :, 1), b_omega(:, :, 1));
lbx = [b_tf(1); opti_lbx(:)];
opti_ubx = cat(3, b_x(:, :, 2), b_y(:, :, 2), b_theta(:, :, 2), b_v(:, :, 2), b_phi(:, :, 2), b_a(:, :, 2), b_omega(:, :, 2));
ubx = [b_tf(2); opti_ubx(:)];

opti_g = [g_kin; g_bctheta; g_tr(:); g_v2v; g_v2o];
lbg = [zeros(size(g_kin)); zeros(size(g_bctheta)); lb_tr(:); zeros(size(g_v2v)); zeros(size(g_v2o))];
ubg = [zeros(size(g_kin)); zeros(size(g_bctheta)); ub_tr(:); inf * ones(size(g_v2v)); inf * ones(size(g_v2o))];
 
opti_x = [x y theta v phi a omega];
opti_x = [tf; opti_x(:)];

nlp = struct('x', opti_x, 'f', f_obj, 'g', opti_g);
S = nlpsol('S', 'ipopt', nlp, struct('ipopt', Params.solver_options));
r = S('x0',x0,'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg);
x_opt = full(r.x);

tf_result = x_opt(1);
results = reshape(x_opt(2:end), [nv, nfe, 7]);
is_ready = S.stats().success;
wall_time = S.stats().t_wall_total;

f_objval = full(r.f);
end