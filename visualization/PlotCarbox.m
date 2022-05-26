function h = PlotCarbox(x, y, theta, varargin)
Params = GetVehicleParams();

carbox = [-Params.Lr -Params.Lb/2; Params.Lw+Params.Lf -Params.Lb/2; Params.Lw+Params.Lf Params.Lb/2; -Params.Lr Params.Lb/2];
carbox = [carbox; carbox(1, :)];

transformed_carbox = [carbox ones(5, 1)] * GetTransformMatrix(x, y, theta)';
h = plot(transformed_carbox(:, 1), transformed_carbox(:, 2), varargin{:});

end
