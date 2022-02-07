function mvtp = LoadMVTP(folder, fileid)

if ~ischar(fileid)
    fileid = num2str(fileid);
end
filename = sprintf("%s/%s.mat", folder, fileid);
load(filename, 'profiles', 'obstacles');

Params = GetModelParams();
mvtp.profiles = profiles(1:Params.nv, :);
mvtp.obstacles = obstacles(1:Params.nobs, :);

end
