function VisualizeStaticInitialGuess(mvtp, x, y, theta)

figure(1)
scr_size = get(0,'screensize');
set(gcf,'outerposition', [1 1 scr_size(4) scr_size(4)]);

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

PlotMVTP(mvtp);

colors = GetRandomColors();
hlines = zeros(Params.nv, 1);
titles = {};
for kk = 1 : Params.nv
    for ii = 1:size(x, 2)
        PlotCarbox(x(kk, ii), y(kk, ii), theta(kk, ii), 'Color', colors(kk,:));
    end
    hlines(kk) = plot(x(kk, :),y(kk, :), 'Color', colors(kk,:), 'LineWidth', 1);
    titles{kk, 1} = sprintf('Vehicle %d', kk);
end

legend(hlines, titles{:}, 'Location', 'northeastoutside', 'NumColumns',1);
end

