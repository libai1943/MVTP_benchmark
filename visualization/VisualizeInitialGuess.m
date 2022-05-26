function VisualizeInitialGuess(mvtp, x, y, theta, varargin)
p = inputParser;
addRequired(p, 'x');
addRequired(p, 'y');
addRequired(p, 'theta');
addOptional(p, 'Tf', 0);
addOptional(p, 'Idx', 1);
addOptional(p, 'Corridors', 0);
addOptional(p, 'ASCO', 0);
addOptional(p, 'Phy', 0);
parse(p, x, y, theta, varargin{:});

Params = GetModelParams();
colorpool = GetRandomColors();

nfe = Params.nfe;
nv = Params.nv;

figure(p.Results.Idx)
scr_size = get(0,'screensize');
set(gcf,'outerposition', [1 1 scr_size(3) scr_size(4)]);

hold on
axis equal
axis tight
box on
grid on

rr = 5;
Params = GetModelParams();
axis([Params.x_min-rr Params.x_max+rr Params.y_min-rr Params.y_max+rr]);

xlabel('X / m', 'fontweight','bold');
ylabel('Y / m', 'fontweight','bold');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);

%% Plotting ASCO Vehicle-to-vehicle, Vehicle-to-obstacle relationship
v2v = cell(nv, nfe);
v2o = cell(nv, nfe);
if iscell(p.Results.ASCO)
    v2v_idx = p.Results.ASCO{1}; v2o_idx = p.Results.ASCO{2};
    for vk = 1:size(v2v_idx, 1)
        [vi_r, vi_c] = ind2sub(size(x), v2v_idx(vk, 1));
        [vj_r, ~] = ind2sub(size(x), v2v_idx(vk, 2));
        v2v{vi_r, vi_c} = [v2v{vi_r, vi_c}; vj_r];
    end
    
    for vk = 1:size(v2o_idx, 1)
        [vi_r, vi_c] = ind2sub(size(x), v2o_idx(vk, 1));
        v2o{vi_r, vi_c} = [v2o{vi_r, vi_c}; v2o_idx(vk, 2)];
    end
end

%% Plotting Animation
pf = 1;
filename = 'animation.gif';

for ii = 1 : nfe
    PlotMVTP(mvtp);

    for kk = 1 : nv
        xc = x(kk,ii);
        yc = y(kk,ii);
        tc = theta(kk,ii);

        PlotCarbox(xc, yc, tc, 'Color', colorpool(kk,:));
        plot(x(kk,1:ii),y(kk,1:ii),'Color', colorpool(kk,:));
        th = text(xc, yc, ['#', num2str(kk)], 'Color', colorpool(kk,:));
        set(th, {'HorizontalAlignment'},{'center'});

        if length(p.Results.Phy) > 1
            pc = p.Results.Phy(kk,ii);
            PlotCarWheels(xc, yc, tc, pc, 'k');
        end
        
        v2v_ki = v2v{kk, ii};
        if v2v_ki
            for j = 1:size(v2v_ki, 1)
                plot([xc, x(v2v_ki(j), ii)], [yc, y(v2v_ki(j), ii)], '--', 'Color', colorpool(kk, :), 'LineWidth', 1);
            end
        end
        
        v2o_ki = v2o{kk, ii};
        if v2o_ki
            for j = 1:size(v2o_ki, 1)
                ob = Params.obstacles(v2o_ki(j), :);
                plot([xc, ob(1)], [yc, ob(2)], ':', 'Color', colorpool(kk, :), 'LineWidth', 1);
            end
        end
    end
    
    drawnow
    h1 = get(gca, 'children');
    
    if p.Results.Tf
        delay = p.Results.Tf / nfe;
        frame = getframe(gca); %get frame
        im = frame2im(frame); 
        [imind,cm] = rgb2ind(im,256); 
        if pf == 1
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime', delay); 
        else 
          imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', delay); 
        end
        pf = pf + 1;
    end

    if (ii ~= nfe)
        delete(h1);
    end
    
end

end