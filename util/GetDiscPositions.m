function [xf, yf, xr, yr] = GetDiscPositions(x, y, theta)
Params = GetVehicleParams();
xf = x + Params.f2x * cos(theta);
xr = x + Params.r2x * cos(theta);
yf = y + Params.f2x * sin(theta);
yr = y + Params.r2x * sin(theta);
end
