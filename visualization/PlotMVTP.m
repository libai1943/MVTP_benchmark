function PlotMVTP(mvtp)

colors = GetRandomColors();
PlotObstacles(mvtp.obstacles);

for kk = 1 : size(mvtp.profiles, 1)
    x0 = mvtp.profiles(kk, 1);
    y0 = mvtp.profiles(kk, 2);
    theta0 = mvtp.profiles(kk, 3);
    xtf = mvtp.profiles(kk, 4);
    ytf = mvtp.profiles(kk, 5);
    thetatf = mvtp.profiles(kk, 6);
    PlotArrow(x0, y0, theta0, colors(kk, :));
    PlotArrow(xtf, ytf, thetatf, colors(kk, :));
    
%     PlotCarbox(x0, y0, theta0,'Color', Params.colors(kk,:));
%     PlotCarbox(xtf, ytf, thetatf,'Color', Params.colors(kk,:));
%     text(x0, y0, ['#', num2str(kk)], 'Color', Params.colors(kk,:));
end
end

function PlotArrow(x, y, theta, color, varargin)
arrow_size = 0.4;
arrow_length = 0.8;
tri = [ 0 arrow_size/2; 0 -arrow_size/2; arrow_length 0; 0 arrow_size/2 ];
points = [tri ones(4, 1)] * GetTransformMatrix(x, y, theta)';
fill(points(:, 1), points(:, 2), color, 'LineStyle', 'none', varargin{:});
end
