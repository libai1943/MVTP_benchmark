% ==============================================================================
% Source Codes for "Fast and Optimal Trajectory Planning for Multiple Vehicles in
% a Nonconvex and Cluttered Environment: Benchmarks, Methodology, and Experiments".
% Yakun Ouyang, Bai Li, Youmin Zhang, et al., 2022 IEEE International Conference on
% Robotics and Automation (ICRA), accepted on Jan. 31, 2022.
% ==============================================================================
% Copyright (C) 2022 Yakun Ouyang and Bai Li. Users MUST cite the related articles
% mentioned in http://grjl.hnu.edu.cn/p/2019256
% License GNU General Public License v3.0
% ==============================================================================
%% Initialize
clear
close all
clc

addpath('util');
addpath('visualization');
addpath('params');

%% Case 1 Demo
mvtp = LoadMVTP('cases', 1);
[solution, metrics] = SolveMVTP(mvtp);

VisualizeInitialGuess(mvtp, solution(:, :, 1), solution(:, :, 2), solution(:, :, 3), 'Phy', solution(:, :, 5));

%% Benchmark
for ii = 1:100
    fprintf("case: %d\n", ii);
    mvtp = LoadMVTP('cases', ii);
    
    [solution, metrics] = SolveMVTP(mvtp);
    save(sprintf('results/benchmark/%d.mat', ii), 'solution', 'metrics');
end

CountMVTP('results/benchmark/*.mat', 'benchmark_result.csv');
