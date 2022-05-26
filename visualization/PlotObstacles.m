function PlotObstacles(obstacles)

for kk = 1 : size(obstacles, 1)
    cur_obs = obstacles(kk, :);
    [obs_x, obs_y] = DrawCircle(cur_obs(1), cur_obs(2), cur_obs(3));
    fill(obs_x, obs_y, [125, 125, 125] ./ 255);
%     text(cur_obs(1), cur_obs(2), ['#', num2str(kk)], 'Color', 'white');
end

end

function [x, y] = DrawCircle(xc, yc, R)
ang = linspace(0, 2 * pi, 360);
x = xc + R .* cos(ang);
y = yc + R .* sin(ang);
end
