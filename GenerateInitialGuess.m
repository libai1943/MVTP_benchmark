function guesses = GenerateInitialGuess(mvtp)
Params = GetModelParams();
VehicleParams = GetVehicleParams();

profiles = mvtp.profiles;
obstacles = mvtp.obstacles;

coarse = zeros(Params.nv, Params.nfe, 3);

config = struct();
config.wheel_base = VehicleParams.Lw;
config.front_hang = VehicleParams.Lf;
config.rear_hang = VehicleParams.Lr;
config.width = VehicleParams.Lb;
config.delta_max = VehicleParams.phi_max;
config.xy_bounds = [Params.x_min, Params.x_max, Params.y_min, Params.y_max];

for ii = 1:Params.nv
    try
        res = cppha(profiles(ii, :), obstacles, config);
        
        if size(res, 1) < Params.nfe
            ha_path = navPath(stateSpaceSE2, res(:, 1:3));
            interpolate(ha_path, Params.nfe);
            coarse(ii, :, :) = ha_path.States;
        else
            pick_idx = linspace(1, size(res, 1), Params.nfe);
            coarse(ii, :, :) = res(floor(pick_idx), 1:3);
        end

    catch
        fprintf('%d HA failed\n', ii);
        
        astar = GenerateAStarPlanner(mvtp.obstacles);
        path = plan(astar, profiles(ii, 1:2), profiles(ii, 4:5), 'world');
        idx = fix(linspace(1, size(path, 1), Params.nfe));
        cur_x = path(idx, 1);
        cur_y = path(idx, 2);
        cur_theta = ones(Params.nfe, 1) * atan2(profiles(ii, 5) - profiles(ii, 2), profiles(ii, 4) - profiles(ii, 1));
        cur_theta(1) = profiles(ii, 3);
        coarse(ii, :, :) = [cur_x, cur_y, cur_theta];
    end
end

guesses = zeros(Params.nv, Params.nfe, 7); % x, y, theta, v, phi, a, omega
real_tf = Params.tf_max;

for ii = 1:Params.nv
    cur_x = coarse(ii, :, 1);
    cur_y = coarse(ii, :, 2);
    cur_theta = ToContinuousAngle(coarse(ii, :, 3));
    [guesses(ii, :, :), tf] = ResamplePath(cur_x, cur_y, cur_theta);

    ratio = tf / real_tf;
    guesses(ii, :, [4 6 7]) = guesses(ii, :, [4 6 7]) * ratio; % scale v, a, omega respectivly
end
end

%%
function planner = GenerateAStarPlanner(obstacles)
Params = GetModelParams();

map = binaryOccupancyMap(Params.x_scale, Params.y_scale, 1 / Params.xy_resolution);
map.LocalOriginInWorld = [Params.x_min, Params.y_min];

xs = Params.x_min:Params.xy_resolution:Params.x_max;
ys = Params.y_min:Params.xy_resolution:Params.y_max;

% From https://www.mathworks.com/matlabcentral/answers/495387-how-to-create-a-filled-circle-within-a-matrix
[mesh_x, mesh_y] = meshgrid(xs, ys);
circle_pixels = zeros(length(ys), length(xs));
for ii = 1:size(obstacles, 1)
    circle_pixels = circle_pixels | (mesh_y - obstacles(ii, 2)).^2 ...
        + (mesh_x - obstacles(ii, 1)).^2 <= obstacles(ii, 3).^2;
end
circle_pixels = flip(circle_pixels, 1);

setOccupancy(map, map.LocalOriginInWorld, circle_pixels);

inflate(map, GetVehicleParams().radius);

planner = plannerAStarGrid(map);
end

%% Generate optimal velocity profile
function [guess, terminal_time] = ResamplePath(cur_x, cur_y, cur_theta)
Params = GetModelParams();
nfe = Params.nfe;

% Judge velocity direction
tracking_angles = atan2(diff(cur_y), diff(cur_x));
tracking_angles = ToContinuousAngle(tracking_angles);
vdr = double(abs(wrapToPi(tracking_angles - cur_theta(1:end-1))) < pi/2);
vdr(vdr==0) = -1;
vdr(end+1) = vdr(end);

spikes = [0 find(abs(diff(vdr)) == 2) nfe];

x1 = [];
y1 = [];
theta1 = [];
v = [];
a = [];
terminal_time = 0;
for ii = 1 : length(spikes)-1
    range = spikes(ii)+1:spikes(ii+1);
    [end_time, x0, y0, theta0, v0, a0] = CalculateSegment(cur_x(range), ...
        cur_y(range), cur_theta(range), vdr(range(end)), terminal_time);
    terminal_time = end_time;
    x1 = [x1, x0(1:end-1)];
    y1 = [y1, y0(1:end-1)];
    theta1 = [theta1, theta0(1:end-1)];
    v = [v, v0(1:end-1)];
    a = [a, a0(1:end-1)];
end
x1 = [x1, x0(end)];
y1 = [y1, y0(end)];
theta1 = [theta1, theta0(end)];
v = [v, 0];
a = [a, 0];

index_sequence = round(linspace(1, length(x1), nfe));
x1 = x1(index_sequence);
y1 = y1(index_sequence);
theta1 = theta1(index_sequence);
v = v(index_sequence);
a = a(index_sequence);

% calculate steering inputs
VehicleParams = GetVehicleParams();

dt = terminal_time / nfe;
dtheta = diff(theta1);

v_next = v(2:end);
phi = atan(dtheta * VehicleParams.Lw ./ (v_next * dt));
phi_nan = find(v_next == 0);
phi_prev = max(1, phi_nan - 1);
phi(phi_nan) = phi(phi_prev);
phi = min(VehicleParams.phi_max, max(-VehicleParams.phi_max, phi));

omega = min(VehicleParams.omega_max, max(-VehicleParams.omega_max, diff(phi) / dt));

phi = [0, phi];
omega = [0, 0, omega];

guess = [x1' y1' theta1' v' phi' a' omega'];
if any(isnan(guess(:)))
    assert(false);
end
end

function [end_time, x0, y0, theta0, v, a] = CalculateSegment(seg_x, seg_y, seg_theta, seg_vdr, begin_time)
% Calculate path length
ds = hypot(diff(seg_x), diff(seg_y));
seg_s = [0 cumsum(ds)];

% remove duplicate entries
[seg_s, ia] = unique(seg_s);
removed_index = setdiff(1:length(seg_x), ia);
seg_x(removed_index) = []; seg_y(removed_index) = []; seg_theta(removed_index) = [];

% Calculate s(t), a(t) and v(t) locally
[s, v, a, terminal_time] = SolveMinTimeOptimalControlProblem(seg_s(end));
% Specify end_time
end_time = terminal_time + begin_time;
% Refine x(t), y(t), and theta(t)
x0 = interp1(seg_s, seg_x, s, 'linear', 'extrap');
y0 = interp1(seg_s, seg_y, s, 'linear', 'extrap');
theta0 = interp1(seg_s, seg_theta, s, 'linear', 'extrap');

if (seg_vdr < 0)
    v = v .* -1;
    a = a .* -1;
end
end

function [s, v, a, terminal_time] = SolveMinTimeOptimalControlProblem(path_length)
Params = GetVehicleParams();
threshold_s = (Params.v_max^2) / Params.a_max;
dt_for_resampling = 0.1;

if (path_length <= threshold_s)
    v_summit = sqrt(path_length * Params.a_max);
    terminal_time = 2 * v_summit / Params.a_max;
    nfe = round(terminal_time / dt_for_resampling);
    time_line = linspace(0, terminal_time, nfe);
    time_vec1 = time_line(time_line <= 0.5 * terminal_time);
    time_vec2 = time_line(time_line > 0.5 * terminal_time);
    
    a = [ones(1,size(time_vec1,2)).* Params.a_max, ones(1,size(time_vec2,2)).* -Params.a_max];
    
    v_part1 = time_vec1 .* Params.a_max;
    v_part2 = v_summit + (time_vec2 - 0.5 * terminal_time).* -Params.a_max;
    v = [v_part1, v_part2];
    
    s_part1 = 0.5 * Params.a_max * (time_vec1.^2);
    s_part2 = 0.5 * path_length + v_summit * (time_vec2 - 0.5 * terminal_time) + 0.5 * -Params.a_max * ((time_vec2 - 0.5 * terminal_time).^2);
    s = [s_part1, s_part2];
else
    s_cruise = path_length - threshold_s;
    time_cruise = s_cruise / Params.v_max;
    time_slope = Params.v_max / Params.a_max;
    terminal_time = 2 * time_slope + time_cruise;
    nfe = round(terminal_time / dt_for_resampling);
    time_line = linspace(0, terminal_time, nfe);
    time_vec1 = time_line(time_line <= time_slope);
    time_vec3 = time_line(time_line > time_slope + time_cruise);
    time_vec1plus2 = time_line(time_line <= time_slope + time_cruise);
    time_vec2 = time_vec1plus2(time_vec1plus2 > time_slope);
    
    a = [ones(1,size(time_vec1,2)).* Params.a_max, zeros(1,size(time_vec2,2)), ones(1,size(time_vec3,2)).* Params.a_max];
    
    v_part1 = time_vec1 .* Params.a_max;
    v_part2 = ones(1,size(time_vec2,2)) .* Params.v_max;
    v_part3 = Params.v_max + (time_vec3 - time_slope - time_cruise).* -Params.a_max;
    v = [v_part1, v_part2, v_part3];
    
    s_part1 = 0.5 * Params.a_max * (time_vec1.^2);
    s_part2 = 0.5 * Params.a_max * (time_slope^2) + ...
        Params.v_max * (time_vec2 - time_slope);
    s_part3 = 0.5 * Params.a_max * (time_slope^2) + Params.v_max * time_cruise + ...
        Params.v_max * (time_vec3 - time_slope - time_cruise) + ...
        0.5 * -Params.a_max * ((time_vec3 - time_slope - time_cruise).^2);
    s = [s_part1, s_part2, s_part3];
end
end

function [angles] = ToContinuousAngle(ang)
angles = ang;
for ii = 2 : length(angles)
    while(angles(ii) - angles(ii-1) > pi + 0.001)
        angles(ii) = angles(ii) - 2 * pi;
    end
    while(angles(ii) - angles(ii-1) < -pi- 0.001)
        angles(ii) = angles(ii) + 2 * pi;
    end
end
end
