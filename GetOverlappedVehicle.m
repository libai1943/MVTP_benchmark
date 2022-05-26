function [collision_occurred, errors] = GetOverlappedVehicle(mvtp, x, y, theta)
Params = GetModelParams();
VehicleParams = GetVehicleParams();

nv = size(x, 1);
nobs = Params.nobs;
nfe = Params.nfe;

collision_occurred = false(nv, nfe);
errors = 1000 * ones(nv, nfe);

for jj = 1 : nv
    for ii = 1 : nfe
        for kk = 1 : nobs
            cur_obs = mvtp.obstacles(kk, :);
            xc = cur_obs(1);
            yc = cur_obs(2);
            rc = cur_obs(3);

            xv = x(jj,ii);
            yv = y(jj,ii);
            tv = theta(jj,ii);

            [xf, yf, xr, yr] = GetDiscPositions(xv, yv, tv);
            
            error = norm([xr - xc, yr - yc]) - rc - VehicleParams.radius;
            if (error < -0.001)
                errors(jj, ii) = min(error, errors(jj, ii));
                collision_occurred(jj, ii) = 1;
            end
            
            error = norm([xf - xc, yf - yc]) - rc - VehicleParams.radius;
            if (error < -0.001)
                errors(jj, ii) = min(error, errors(jj, ii));
                collision_occurred(jj, ii) = 1;
            end
        end

        for kk = (jj + 1) : nv
            xv1 = x(jj,ii);
            yv1 = y(jj,ii);
            tv1 = theta(jj,ii);
            xv2 = x(kk,ii);
            yv2 = y(kk,ii);
            tv2 = theta(kk,ii);
                        
            [collided, error] = IsConfig1CollidingWithConfig2([xv1, yv1, tv1], [xv2, yv2, tv2]);
            if (collided)
                errors(jj, ii) = min(error, errors(jj, ii));
                collision_occurred([jj kk], ii) = 1;
            end
        end
    end
end

collision_occurred = any(collision_occurred, 2);

end

function [is_colliding, error] = IsConfig1CollidingWithConfig2(vec1, vec2)

[xf, yf, xr, yr] = GetDiscPositions([vec1(1), vec2(1)], [vec1(2), vec2(2)], [vec1(3), vec2(3)]);
P11 = [xr(1), yr(1)];
P12 = [xf(1), yf(1)];
P21 = [xr(2), yr(2)];
P22 = [xf(2), yf(2)];
error = min([norm(P11 - P21), norm(P11 - P22), norm(P12 - P21), norm(P12 - P22)]) - 2 * GetVehicleParams().radius;

if (error <= -0.001)
    is_colliding = 1;
else
    is_colliding = 0;
end
end