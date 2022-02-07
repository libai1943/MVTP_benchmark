function CountMVTP(file_pattern, output_file)
files = dir(file_pattern);
 
fp = fopen(output_file, 'w');
fprintf(fp, 'name,iter,ready,tf,time,obj\n');
 
for ii = 1:size(files, 1)
    if files(ii).isdir
        continue
    end
    
    filepath = [files(ii).folder '/' files(ii).name];
    
    load(filepath, 'solution', 'metrics');
    run_iter = 1;
    if isfield(metrics, 'iter')
        run_iter = metrics.iter;
    end
    is_ready = ~isempty(solution);
    
    final_tf = 0.0;
    if isfield(metrics, 'tf')
        final_tf = metrics.tf;
    end
    
    run_time = metrics.total_time;
    final_obj = 0.0;
    if isfield(metrics, 'obj')
        final_obj = metrics.obj;
    elseif isfield(metrics, 'f_objval')
        final_obj = metrics.f_objval;
    end
    
    fprintf(fp, '%s,%d,%d,%f,%f,%f\n', files(ii).name, run_iter, is_ready, final_tf, run_time, final_obj);
end

fclose(fp);
end
