function SaveMVTP(fileid, profiles, obstacles)
save(sprintf("cases/%d.mat", fileid), 'profiles', 'obstacles');
end
