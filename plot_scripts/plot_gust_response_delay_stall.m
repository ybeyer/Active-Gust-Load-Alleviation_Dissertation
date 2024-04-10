clear all;

% add folders to path
addPath();

is_tikz_export_desired = false;

%% Simulink model initialization

model = 'sim_flexible_unsteady_indi';

% flight point
% fp_spec.Altitude	= 6000; % m
% fp_spec.EAS         = 177; % m/s
fp_spec.Altitude	= 11200; % m
fp_spec.EAS         = 121.8; % m/s
% fp_spec.Altitude	= 6000; % m
% fp_spec.EAS         = 177; % m/s

[Ma,~] = altEas2MaTas( fp_spec.Altitude, fp_spec.EAS );

% aircraft parameters
[aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', true, 'Mach', Ma, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true, 'ControlsMainFile', 'wingControls_params_mainTefRedCm' );

% environment parameters
envir = envirLoadParams('envir_params_default');

% Temporary airplane pitch rate controller
pitch_rate_controller = loadParams( 'pitchRateController_params_se2a' );

% GLA INDI controller parameters
gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', 500, ...
    'BoostServos', 3, 'BoostUnstAeroMin', 3, 'BoostUnstAeroMax', 8 );

% Trim aircraft
tp_indi = trimFlexibleUnsteady(aircraft,structure,fp_spec,model);

% init controller steady state
tp_indi = tpInitController(tp_indi,gla_indi);

ic = tpGenerateIC(tp_indi);

%% Run simulations

tic

gla_indi = glaIndiCreate( aircraft, fp_spec );

time = 1.1;

gust_grad_dist  = 180;
is_failure      = false;
is_gla_enabled  = true;
omega_sens      = 500;

aircraft.wing_main.config.is_stall = 0;
simout_open_loop = simGust(gust_grad_dist,time,false,is_failure);
aircraft.wing_main.config.is_stall = 1;
simout_open_loop_stall = simGust(gust_grad_dist,time,false,is_failure);

T_delay = [0.16,0.08,0.04,0.02,0.01];
simout  = {};

for i = 1:length(T_delay)
    
    [boost_servo,boost_aero] = delay2Booster( T_delay(i), omega_sens, gla_indi.dtc, gla_indi.atc );

    gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
        'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8 );
    gla_indi.ca.gamma = 100;
    gla_indi.ca.W_v(1,1) = 0.006;
    gla_indi.ca.W_v(3,3) = 1e-9;
    gla_indi.ca.W_u=diag(ones(1,38));

    aircraft.wing_main.config.is_stall = 0;
    simout{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);
    aircraft.wing_main.config.is_stall = 1;
    simout_stall{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);
    
end

toc

%% 
ds = 5;
Legend = {};
figtmp = figure;
h = plot([0,1],rand(2,length(T_delay)+1));
colors = {h.Color};
close(figtmp)
h = {};
fig1=figure;
hold on
h_ol = plot(simout_open_loop_stall.WBM.Time(1:ds:end),simout_open_loop_stall.WBM.Data(1:ds:end,5)/simout_open_loop_stall.WBM.Data(1,5),'Color',colors{1},'LineStyle','-');
plot(simout_open_loop.WBM.Time(1:ds:end),simout_open_loop.WBM.Data(1:ds:end,5)/simout_open_loop.WBM.Data(1,5),'Color',colors{1},'LineStyle','--');
for i = 1:length(T_delay)
    h{i} = plot(simout_stall{i}.WBM.Time(1:ds:end),simout_stall{i}.WBM.Data(1:ds:end,5)/simout_stall{i}.WBM.Data(1,5),'Color',colors{i+1},'LineStyle','-');
    plot(simout{i}.WBM.Time(1:ds:end),simout{i}.WBM.Data(1:ds:end,5)/simout{i}.WBM.Data(1,5),'Color',colors{i+1},'LineStyle','--');
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end
xlim([0,time])
xlabel('Time, s')
ylabel('Relative WRBM')
grid on
box on

legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex')


%% 
ds = 5;
h = {};
Legend = {};
fig2=figure;
hold on
h_ol = plot(simout_open_loop_stall.acc.Time(1:ds:end),-simout_open_loop_stall.acc.Data(1:ds:end)/9.81+1,'Color',colors{1},'LineStyle','-');
plot(simout_open_loop.acc.Time(1:ds:end),-simout_open_loop.acc.Data(1:ds:end)/9.81+1,'Color',colors{1},'LineStyle','--');
for i = 1:length(T_delay)
    h{i} = plot(simout_stall{i}.acc.Time(1:ds:end),-simout_stall{i}.acc.Data(1:ds:end)/9.81+1,'Color',colors{i+1},'LineStyle','-');
    plot(simout{i}.acc.Time(1:ds:end),-simout{i}.acc.Data(1:ds:end)/9.81+1,'Color',colors{i+1},'LineStyle','--');
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end
xlim([0,time])
xlabel('Time, s')
ylabel('Load factor')
grid on
box on

ax1 = get(fig1,'CurrentAxes');
ax2 = get(fig2,'CurrentAxes');
if ax2.YLim(2) > ax1.YLim(2)
    ax1.YLim(2) = ax2.YLim(2);
else
    ax2.YLim(2) = ax1.YLim(2);
end
if ax2.YLim(1) < ax1.YLim(1)
    ax1.YLim(1) = ax2.YLim(1);
else
    ax2.YLim(1) = ax1.YLim(1);
end

% legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex')


%%
fig_temp = figure;
h = plot(rand(2,3),rand(2,3));
c = get(h,'Color');
colors_default = reshape([c{:}],3,[]);
close(fig_temp);

fig3 = figure;
hold on
xlabel('Time, s')
ylabel('Flap commands')
xlim([0,time])
fig4 = figure;
hold on
xlabel('Time, s')
ylabel('Flap commands')
xlim([0,time])

idx_flap_leg = 1:6:19;
idx_flap = 1:19;
ds = 5;

Legend = {};
hl = {};

colors = interp1([1,9,19]',colors_default(:,end:-1:1)',1:19)';
for i = 1:length(idx_flap)
    figure(fig3);
    h = plot(simout{end-1}.u.Time(1:ds:end),simout{end-1}.u.Data(1:ds:end,idx_flap(i)),'Color',colors(:,i));
    figure(fig4);
    plot(simout_stall{end-1}.u.Time(1:ds:end),simout_stall{end-1}.u.Data(1:ds:end,idx_flap(i)),'Color',colors(:,i));
    if any(i==idx_flap_leg)
        hl{end+1} = h;
        Legend{end+1} = ['Strip ',num2str(idx_flap(i))];
    end
end

ax1 = get(fig3,'CurrentAxes');
ax2 = get(fig4,'CurrentAxes');
if ax2.YLim(2) > ax1.YLim(2)
    ax1.YLim(2) = ax2.YLim(2);
else
    ax2.YLim(2) = ax1.YLim(2);
end
if ax2.YLim(1) < ax1.YLim(1)
    ax1.YLim(1) = ax2.YLim(1);
else
    ax2.YLim(1) = ax1.YLim(1);
end

figure(fig3);
legend([hl{:}],Legend{:},'location','southeast')
grid on
box on

figure(fig4);
grid on
box on

%%
fig5=figure;
hold on
xlabel('Time, s')
ylabel('Local lift coefficient')
xlim([0,time])

fig6=figure;
hold on
xlabel('Time, s')
ylabel('Local lift coefficient')
xlim([0,time])

idx_flap_leg = 1:6:19;
idx_flap = 1:19;

Legend = {};
hl = {};
for i = 1:length(idx_flap)
    figure(fig5);
	h=plot(simout{end-1}.c_L.Time(1:ds:end),squeeze(simout{end-1}.c_L.Data(1,idx_flap(i),1:ds:end))','Color',colors(:,i));
    figure(fig6);
    plot(simout_stall{end-1}.c_L.Time(1:ds:end),squeeze(simout_stall{end-1}.c_L.Data(1,idx_flap(i),1:ds:end))','Color',colors(:,i));
    if any(i==idx_flap_leg)
        hl{end+1} = h;
        Legend{end+1} = ['Strip ',num2str(idx_flap(i))];
    end
end

ax1 = get(fig5,'CurrentAxes');
ax2 = get(fig6,'CurrentAxes');
if ax2.YLim(2) > ax1.YLim(2)
    ax1.YLim(2) = ax2.YLim(2);
else
    ax2.YLim(2) = ax1.YLim(2);
end
ax1.YLim(1) = 0;
ax2.YLim(1) = 0;

figure(fig5);
legend([hl{:}],Legend{:},'location','south')
grid on
box on

figure(fig6);
grid on
box on

%%
fig7=figure;
hold on
xlabel('Time, s')
ylabel('Flow separation point')
xlim([0,time])
ylim([0,1])
grid on
box on

fig8=figure;
hold on
xlabel('Time, s')
ylabel('Flow separation point')
xlim([0,time])
ylim([0,1])
grid on
box on

idx_flap_leg = 1:6:19;
idx_flap = 1:19;

Legend = {};
hl = {};
for i = 1:length(idx_flap)
    figure(fig7);
    h=plot(simout_open_loop_stall.x_f.Time(1:ds:end),squeeze(simout_open_loop_stall.x_f.Data(1,idx_flap(i),1:ds:end))','Color',colors(:,i));
    figure(fig8);
    plot(simout_stall{end-1}.x_f.Time(1:ds:end),squeeze(simout_stall{end-1}.x_f.Data(1,idx_flap(i),1:ds:end))','Color',colors(:,i));
    if any(i==idx_flap_leg)
        hl{end+1} = h;
        Legend{end+1} = ['Strip ',num2str(idx_flap(i))];
    end
end

legend([hl{:}],Legend{:},'location','northeast')



num = [];
% num = '_2';

%% Export figure to TikZ
figure(fig1)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename(['gust_response_delay_wrbm_stall',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig2)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename(['gust_response_delay_acc_stall',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig3)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename(['gust_control_input_wo_stall',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig4)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename(['gust_control_input_w_stall',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig5)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=2'};
    filename = exportFilename(['gust_local_c_L_wo_stall',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig6)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=2'};
    filename = exportFilename(['gust_local_c_L_w_stall',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig7)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename(['gust_stall_local_xf_ol',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig8)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename(['gust_stall_local_xf_cl',num,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
