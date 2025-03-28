clear all;

% add folders to path
is_tigl_installed = addPath();

is_tikz_export_desired = false;

%% Simulink model initialization

model = 'sim_flexible_unsteady_indi';

% flight point
fp_spec.Altitude	= 6000; % m
fp_spec.EAS         = 177; % m/s

[Ma,~] = altEas2MaTas( fp_spec.Altitude, fp_spec.EAS );

% aircraft parameters
if is_tigl_installed
    [aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', false, 'Mach', Ma, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true, 'ControlsMainFile', 'wingControls_params_mainTefRedCm' );
else
    load('data/aircraft_structure.mat');
    wingSetCustomActuatorPath(aircraft.wing_main);
end

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

time = 1.0;

gust_grad_dist  = 180;
is_failure      = false;
is_gla_enabled  = true;
omega_sens      = 500;

simout_open_loop = simGust(gust_grad_dist,time,false,is_failure);

T_delay = [0.16,0.08,0.04,0.02,0.01];

k_rel = linspace(0.1,1,7);

k = {};

simout  = {};

open(model);
set_param(model,'SimulationCommand','update');
set_param(model,"FastRestart","on");

for i = 1:length(T_delay)
    
    for j = 1:length(k_rel)
    
        [boost_servo,boost_aero] = delay2Booster( T_delay(i), omega_sens, gla_indi.dtc, gla_indi.atc );
        
        % de-activate flap rate limit
        aircraft.actuators.LAD.defl_rate_max = 100;
        aircraft.actuators.LAD.defl_rate_min = -100;

        gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
            'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8 );
        gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
        
        gla_indi.k.pos = k_rel(j) * gla_indi.k.pos;
        gla_indi.k.vel = k_rel(j) * gla_indi.k.vel;
        gla_indi.k.acc = k_rel(j) * gla_indi.k.acc;
        k{i,j} = [gla_indi.k.pos,gla_indi.k.vel,gla_indi.k.acc];

        simout{i,j} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);
        
    end
    
end

set_param(model,"FastRestart","off");

%% Post process simulations

WRBM_rel_ol = max(simout_open_loop.WBM.Data(:,5)/simout_open_loop.WBM.Data(1,5));
WRBM_rel = [];
Load_factor_ol = max(-simout_open_loop.acc.Data)/9.81 + 1;
Load_factor = [];
k_pos = [];
Legend = {};
for i = 1:length(T_delay)
    for j = 1:length(k_rel)
        WRBM_rel(i,j) = 1 + max(abs(simout{i,j}.WBM.Data(:,5)-simout{i,j}.WBM.Data(1,5))/simout{i,j}.WBM.Data(1,5));
        Load_factor(i,j) = 1 + max(abs(-simout{i,j}.acc.Data)/9.81);
        k_pos(i,j) = k{i,j}(1);
    end
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

%% Plot max. relative WRBM over feedback gain
fig1=figure;
h_ol = semilogx([min(k_pos(k_pos>0)),max(k_pos(:))],repmat(WRBM_rel_ol,1,2),'-');
hold on
h = semilogx(k_pos',WRBM_rel','-');
grid on
xlabel('Mode feedback gain $k_{\eta_1}$','interpreter','latex')
ylabel('Max. relative WRBM')
legend([h_ol;h],'Open loop',Legend{:},'interpreter','latex','location','northeast')

%% Plot max. load factor over feedback gain
fig2=figure;
h_ol = semilogx([min(k_pos(k_pos>0)),max(k_pos(:))],repmat(Load_factor_ol,1,2),'-');
hold on
h = semilogx(k_pos',Load_factor','-');
ylim([1,inf])
grid on
xlabel('Mode feedback gain $k_{\eta_1}$','interpreter','latex')
ylabel('Max. load factor')

ax1 = get(fig1,'CurrentAxes');
ax2 = get(fig2,'CurrentAxes');
if ax1.YLim(2) == Inf
    ax1.YLim(2) = round(WRBM_rel_ol,1) + 0.1;
end
if ax2.YLim(2) == Inf
    ax2.YLim(2) = round(Load_factor_ol,1) + 0.1;
end
if ax2.YLim(2) > ax1.YLim(2)
    ax1.YLim(2) = ax2.YLim(2);
else
    ax2.YLim(2) = ax1.YLim(2);
end
ax1.YLim(1) = 1;
ax2.YLim(1) = 1;

%% Export figure to TikZ
figure(fig1)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_delay_gain_wrbm.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Export figure to TikZ
figure(fig2)
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_delay_gain_acc.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

