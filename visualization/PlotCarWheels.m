function hs = PlotCarWheels(x, y, theta, phy, varargin)
Params = GetVehicleParams();

wheel_box = [-Params.wheel_radius -Params.wheel_width / 2;
    +Params.wheel_radius -Params.wheel_width / 2;
    +Params.wheel_radius +Params.wheel_width / 2;
    -Params.wheel_radius +Params.wheel_width / 2];

front_x = x + Params.Lw * cos(theta);
front_y = y + Params.Lw * sin(theta);
track_width_2 = (Params.Lb - Params.wheel_width / 2) / 2;

boxes = {
    TransformBox(wheel_box, x - track_width_2 * sin(theta), y + track_width_2 * cos(theta), theta);
    TransformBox(wheel_box, x + track_width_2 * sin(theta), y - track_width_2 * cos(theta), theta);
    TransformBox(wheel_box, front_x - track_width_2 * sin(theta), front_y + track_width_2 * cos(theta), theta+phy);
    TransformBox(wheel_box, front_x + track_width_2 * sin(theta), front_y - track_width_2 * cos(theta), theta+phy);
};

hs = cell(4, 1);
for ii = 1:4
    hs{ii} = fill(boxes{ii}(:, 1), boxes{ii}(:, 2), varargin{:});
end

end

function transformed = TransformBox(box, x, y, theta)
transformed = [box; box(1, :)];
transformed = [transformed ones(5, 1)] * GetTransformMatrix(x, y, theta)';
transformed = transformed(:, 1:2);
end