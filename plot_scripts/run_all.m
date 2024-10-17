% Run all plot scripts within this folder
% (the scripts are partly run multiple times)

addPath();

runAll();

function runAll()
    listing = dir;
    num_files = length(listing);
    for i = 19:num_files
        filename = listing(i).name;
        if contains(filename,'plot_')
            file_str = activateTikz(filename);
            if isequal(filename,'plot_gust_envelope_delay_rate_limit.m')
                runPlotGustEnvelope('plot_gust_envelope_delay_rate_limit.m');
            elseif isequal(filename,'plot_gust_response_delay_stall.m')
                runPlotGustStall('plot_gust_response_delay_stall.m');
            else
                runScript(filename);
            end
            restoreFile(filename,file_str);
            close all;
        end
        disp(['Progress: ',num2str(i/num_files*100),'%']);
    end
end

function runScript(filename)
    evalin('base',['run("',filename,'")']);
end

function runPlotGustEnvelope(filename)
    opts = [-1,0,1,2,4,5,6];
    for j = opts
        file_str = fileread(filename);
        file_str_tmp = strrep(file_str,'cntrl_var = 2;',['cntrl_var = ',num2str(j),';']);
        fid = fopen(filename,'w');
        fprintf(fid,'%s\n',file_str_tmp);
        fclose(fid);
        evalin('base',['run("',filename,'")']);
        fid = fopen(filename,'w');
        fprintf(fid,'%s\n',file_str);
        fclose(fid);
    end
end

function runPlotGustStall(filename)
    evalin('base',['run("',filename,'")']);
    file_str = fileread(filename);
    file_str_tmp = strrep(file_str,'fp_spec.Altitude	= 11200; % m','fp_spec.Altitude	= 6000; % m');
    file_str_tmp = strrep(file_str_tmp,'fp_spec.EAS         = 121.8; % m/s','fp_spec.EAS         = 177; % m/s');
    fid = fopen(filename,'w');
    fprintf(fid,'%s\n',file_str_tmp);
    fclose(fid);
    evalin('base',['run("',filename,'")']);
    fid = fopen(filename,'w');
    fprintf(fid,'%s\n',file_str);
    fclose(fid);
end

function file_str = activateTikz(filename)
    file_str = fileread(filename);
    file_str_tmp = strrep(file_str,'is_tikz_export_desired = false','is_tikz_export_desired = true');
    fid = fopen(filename,'w');
    fprintf(fid,'%s\n',file_str_tmp);
    fclose(fid);
end

function restoreFile(filename,file_str)
    evalin('base',['run("',filename,'")']);
    fid = fopen(filename,'w');
    fprintf(fid,'%s\n',file_str);
    fclose(fid);
end
