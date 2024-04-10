clear all;

% add folders to path
addPath();

is_tikz_export_desired = false;

%% Simulink model initialization

model = 'sim_flexible_unsteady_indi_turbulence';

% flight point
fp_spec.Altitude	= 6000; % m
fp_spec.EAS         = 177; % m/s

[Ma,~] = altEas2MaTas( fp_spec.Altitude, fp_spec.EAS );

% aircraft parameters
[aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', false, 'Mach', Ma, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true, 'ControlsMainFile', 'wingControls_params_mainTefRedCm' );

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

time = 12;

turbulence_rms  = 6.5;
% turbulence_rms  = 7.9;
is_failure      = false;
is_gla_enabled  = true;
omega_sens      = 500;

simout_open_loop = simTurbulence(turbulence_rms,time,false,is_failure);

T_delay = [0.16,0.08,0.04,0.02,0.01];
simout  = {};
simout_rl = {};

%%

for i = 1:length(T_delay)
    
    [boost_servo,boost_aero] = delay2Booster( T_delay(i), omega_sens, gla_indi.dtc, gla_indi.atc );
    
    aircraft.actuators.LAD.defl_rate_max = 100;
    aircraft.actuators.LAD.defl_rate_min = -100;

    gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
        'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8, ...
        'ModeControlIdx', [1,7], 'WeightModes', [1,1e-9]);
    gla_indi.ca.W_v(1,1) = 1e-9;
    gla_indi.ca.W_v(2,2) = 1;
    gla_indi.ca.W_v(3,3) = 1e-9;
    gla_indi.ca.gamma = 1e7;
    gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));

    simout{i} = simTurbulence(turbulence_rms,time,is_gla_enabled,is_failure);
    
    aircraft.actuators.LAD.defl_rate_max = deg2rad(100);
    aircraft.actuators.LAD.defl_rate_min = -deg2rad(100);
    
    gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
        'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8, ...
        'ModeControlIdx', [1,7], 'WeightModes', [1,1e-9] );
    gla_indi.ca.W_v(1,1) = 1e-9;
    gla_indi.ca.W_v(2,2) = 1;
    gla_indi.ca.W_v(3,3) = 1e-9;
    gla_indi.ca.gamma = 1e7;
    gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
    
    simout_rl{i} = simTurbulence(turbulence_rms,time,is_gla_enabled,is_failure);
    
end

%% Plot 1
ds = 5;
h = {};
Legend = {};
fig1=figure;
hold on
h_ol = plot(simout_open_loop.WBM.Time(1:ds:end),simout_open_loop.WBM.Data(1:ds:end,5)/simout_open_loop.WBM.Data(1,5));
for i = 1:length(T_delay)
    h{i} = plot(simout{i}.WBM.Time(1:ds:end),simout{i}.WBM.Data(1:ds:end,5)/simout{i}.WBM.Data(1,5));
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

xlabel('Time, s')
ylabel('Relative WRBM')
grid on
box on

legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex','location','northwest')


%% Plot 2
ds = 2;
h = {};
Legend = {};
fig2=figure;
hold on
h_ol = plot(simout_open_loop.acc.Time(1:ds:end),-simout_open_loop.acc.Data(1:ds:end)/9.81+1);
for i = 1:length(T_delay)
    h{i} = plot(simout{i}.acc.Time(1:ds:end),-simout{i}.acc.Data(1:ds:end)/9.81+1);
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

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

% legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex','location','northwest')

%% Plot 3
ds = 5;
h = {};
Legend = {};
fig3=figure;
hold on
h_ol = plot(simout_open_loop.eta.Time(1:ds:end),simout_open_loop.eta.Data(1:ds:end,7)/simout_open_loop.eta.Data(1,7));
for i = 1:length(T_delay)
    h{i} = plot(simout{i}.eta.Time(1:ds:end),simout{i}.eta.Data(1:ds:end,7)/simout{i}.eta.Data(1,7));
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

xlabel('Time, s')
ylabel('Relative mode deflection')
grid on
box on

ax1 = get(fig1,'CurrentAxes');
ax2 = get(fig3,'CurrentAxes');
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

% legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex','location','northwest')

%% Plot 4
ds = 5;
defl0 = 0;
defl_ol = -aircraft.wing_main.aeroelasticity.T_cs(3,:)*simout_open_loop.eta.Data';
h = {};
Legend = {};
fig4=figure;
hold on
h_ol = plot(simout_open_loop.eta.Time(1:ds:end),(defl_ol(1:ds:end)-defl0)/(defl_ol(1)-defl0));
for i = 1:length(T_delay)
    defl = -aircraft.wing_main.aeroelasticity.T_cs(3,:)*simout{i}.eta.Data';
    h{i} = plot(simout{i}.eta.Time(1:ds:end),(defl(1:ds:end)-defl0)/(defl(1)-defl0));
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

xlabel('Time, s')
ylabel('Relative tip deflection')
grid on
box on

% legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex','location','northwest')

%% Plot 5
ds = 8;
xmax = 2800;
fig5 = figure;
ax1 = gca;
[X,Y] = meshgrid( 0:ds:xmax, aircraft.wing_main.geometry.ctrl_pt.pos(2,:) );
% surf(X,Y,wind2D(1:ds:xmax+1,1:40)'+50,'EdgeAlpha',0)
imagesc(0:ds:xmax,aircraft.wing_main.geometry.ctrl_pt.pos(2,:),wind2D(1:ds:xmax+1,1:40)')
% view(0,90)
xticks(0:500:xmax);
xlim([min(min(X)),max(max(X))])
ylim([min(min(Y)),max(max(Y))])
box on
xlabel('Longitudinal position, m')
ylabel('Lateral position, m')
c = colorbar('NorthOutside');
c.Label.String = 'Wind velocity, m/s';
c.Ticks = c.Ticks + 50;
% set(ax1,'Layer','Top')
% grid off

%% Plot 6
ds = 2;
tmax = 12;
fig6 = figure;
plot( linspace(0,xmax,round(length(wind2D(1:ds:xmax+1,20)))), wind2D(1:ds:xmax+1,20), 'k-' )
box on
grid on
xticks(0:500:xmax);
xlabel('Longitudinal position, m')
ylabel('Wind velocity, m/s')
% xticks(0:500:xmax);
xlim([0,xmax])

%% Plot 7
fig7=figure;
hold on
WRBM_max = [];
WRBM_max_rl = [];
for i = 1:length(T_delay)-1
    WRBM_max(i) = max(simout{i}.WBM.Data(:,5)/simout{i}.WBM.Data(1,5));
    WRBM_max_rl(i) = max(simout_rl{i}.WBM.Data(:,5)/simout_rl{i}.WBM.Data(1,5));
end
plot(T_delay(1:end-1),WRBM_max_rl,'k-')
plot(T_delay(1:end-1),WRBM_max,'k--')
xlim([0,max(T_delay)])
ylim([1,max(-simout_open_loop.acc.Data)/9.81+1])
xlabel('Control system time delay, s')
ylabel('Max. relative WRBM')
grid on
box on
legend('With rate limit','Without rate limit','location','southeast')

%% Plot 8
fig8=figure;
hold on
acc_max = [];
acc_max_rl = [];
for i = 1:length(T_delay)-1
    acc_max(i) = max(-simout{i}.acc.Data/9.81+1);
    acc_max_rl(i) = max(-simout_rl{i}.acc.Data/9.81+1);
end
plot(T_delay(1:end-1),acc_max_rl,'k-')
plot(T_delay(1:end-1),acc_max,'k--')
xlim([0,max(T_delay)])
ylim([1,max(-simout_open_loop.acc.Data)/9.81+1])
xlabel('Time delay, s')
ylabel('Max. load factor')
grid on
box on
legend('With rate limit','Without rate limit','location','southeast')

%% Export figure to TikZ
figure(fig1)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=3'};
    filename = exportFilename('turbulence_response_delay_wrbm.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig2)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=3'};
    filename = exportFilename('turbulence_response_delay_acc.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig3)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=3'};
    filename = exportFilename('turbulence_response_delay_eta1.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig4)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=3'};
    filename = exportFilename('turbulence_response_delay_tip.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig5)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=3'};
    filename = exportFilename('turbulence_wind_field.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig6)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=3'};
    filename = exportFilename('turbulence_wind_velocity.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig7)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('turbulence_wrbm_rate.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig8)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('turbulence_acc_rate.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
