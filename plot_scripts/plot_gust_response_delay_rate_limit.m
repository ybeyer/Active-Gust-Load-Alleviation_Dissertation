clear all;

% add folders to path
addPath();

is_tikz_export_desired = false;

%% Simulink model initialization

model = 'sim_flexible_unsteady_indi';

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

time = 0.8;

gust_grad_dist  = [30,45,67.5,101.25,151.875,227.8125,350];
is_failure      = false;
is_gla_enabled  = true;
omega_sens      = 500;

T_delay = [0.16,0.08,0.04,0.02,0.01];
simout  = {};
simout_nrl = {};

set_param(model,"FastRestart","on");

for i = 1:length(T_delay)
    
    for j = 1:length(gust_grad_dist)
    
        [boost_servo,boost_aero] = delay2Booster( T_delay(i), omega_sens, gla_indi.dtc, gla_indi.atc );
        
        aircraft.actuators.LAD.defl_rate_max = deg2rad(10000);
        aircraft.actuators.LAD.defl_rate_min = -deg2rad(10000);

        gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
            'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8 );
        gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
        
%         % Paper
%         gla_indi.ca.W_v(1,1) = 2.5e-3;
%         gla_indi.ca.W_v(3,3) = 1e-9;
%         gla_indi.ca.gamma = 100;

        simout_nrl{i,j} = simGust(gust_grad_dist(j),time,is_gla_enabled,is_failure);
        
        aircraft.actuators.LAD.defl_rate_max = deg2rad(100);
        aircraft.actuators.LAD.defl_rate_min = -deg2rad(100);
        
        gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
            'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, 'BoostUnstAeroMax', 8 );
        gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
        
%         % Paper
%         gla_indi.ca.W_v(1,1) = 2.5e-3;
%         gla_indi.ca.W_v(3,3) = 1e-9;
%         gla_indi.ca.gamma = 100;
        
        simout{i,j} = simGust(gust_grad_dist(j),time,is_gla_enabled,is_failure);
        
    end
    
end

set_param(model,"FastRestart","off");

%%
WRBM_rel = [];
Load_factor = [];
WRBM_rel_nrl = [];
Load_factor_nrl = [];
gust_grad = linspace(min(gust_grad_dist),max(gust_grad_dist),100);
gust_grad_max = [];
gust_grad_max_nrl = [];
WRBM_max = [];
WRBM_max_nrl = [];
Legend = {};
for i = 1:length(T_delay)
    for j = 1:length(gust_grad_dist)
        WRBM_rel(i,j) = 1 + max(abs(simout{i,j}.WBM.Data(:,5)-simout{i,j}.WBM.Data(1,5))/simout{i,j}.WBM.Data(1,5));
        Load_factor(i,j) = 1 + max(abs(-simout{i,j}.acc.Data/9.81));
        WRBM_rel_nrl(i,j) = 1 + max(abs(simout_nrl{i,j}.WBM.Data(:,5)-simout_nrl{i,j}.WBM.Data(1,5))/simout_nrl{i,j}.WBM.Data(1,5));
        Load_factor_nrl(i,j) = 1 + max(abs(-simout_nrl{i,j}.acc.Data/9.81));
    end
    WRBM_gust_grad = interp1( gust_grad_dist, WRBM_rel(i,:), gust_grad, 'makima' );
    [WRBM_max(i),gust_grad_max_idx] = max( WRBM_gust_grad );
    gust_grad_max(i) = gust_grad(gust_grad_max_idx);
    WRBM_gust_grad_nrl = interp1( gust_grad_dist, WRBM_rel_nrl(i,:), gust_grad, 'makima' );
    [WRBM_max_nrl(i),gust_grad_max_idx_nrl] = max( WRBM_gust_grad_nrl );
    gust_grad_max_nrl(i) = gust_grad(gust_grad_max_idx_nrl);
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

%%

figure
hold on
h1=plot(gust_grad_dist,WRBM_rel);
for i = 1:length(h1)
    plot(gust_grad_dist,WRBM_rel_nrl(i,:),'LineStyle','--','Color',h1(i).Color)
end
grid on

xlabel('Gust gradient distance, ft')
ylabel('Max. relative WRBM')

%% 
ds = 10;
h = {};
Legend = {};
figure
hold on
h_ol = plot(simout_open_loop.acc.Time(1:ds:end),-simout_open_loop.acc.Data(1:ds:end)/9.81+1);
for i = 1:length(T_delay)
    h{i} = plot(simout{i}.acc.Time(1:ds:end),-simout{i}.acc.Data(1:ds:end)/9.81+1);
    Legend{i} = ['$T=',num2str(T_delay(i)),'$\,s'];
end

xlabel('Time, s')
ylabel('Max. load factor')
grid on
box on

legend([h_ol,h{:}],'Open loop',Legend{:},'interpreter','latex')

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('gust_response_delay_acc.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
