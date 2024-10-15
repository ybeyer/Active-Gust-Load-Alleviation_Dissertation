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

T_delay = 100;
T_delay_sens = 2/omega_sens;
T_delay_act = T_delay - T_delay_sens;
T_delay_act_min = min( gla_indi.dtc, gla_indi.atc(1) );
boost_servo = gla_indi.dtc / (T_delay_act/2);
boost_aero = gla_indi.atc(1) / (T_delay_act/2);
gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
    'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, ...
    'ModeControlIdx', [1,7], 'WeightModes', [1,0.001] );

simout_open_loop = simGust(gust_grad_dist,time,false,is_failure);

%%
simout1={};
simout2={};
simout3={};
simout4={};
simout5={};
simout6={};

%% Run simulations

gla_indi = glaIndiCreate( aircraft, fp_spec );

fm_atti_pitch.sens_filt.omega = gla_indi.sflt.omega;
aircraft.actuators.LAD.defl_rate_max = 100;
aircraft.actuators.LAD.defl_rate_min = -100;

T_delay = 0.02;

tic;
open(model);
set_param(model,'SimulationCommand','update');
set_param(model,"FastRestart","on");

for i = 1:2
    
    if i == 1
        boost_servo = 1;
        boost_aero = 1;
    else
        [boost_servo,boost_aero] = delay2Booster( T_delay, omega_sens, gla_indi.dtc, gla_indi.atc );
    end

    % a_z, eta1, eta2
    gla_indi = glaIndiCreate( aircraft, fp_spec, 'SensOmega', omega_sens, ...
        'BoostServos', boost_servo, 'BoostUnstAeroMin', boost_aero, ...
        'ModeControlIdx', [1,7], 'WeightModes', [1,0.001] );
    if i == 2
        gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));
    end
    gla_indi.ca.W_v(1,1) = 1e-3;
    simout1{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);

    % a_z, eta1
    gla_indi.ca.W_v(3,3) = 1e-9;
    simout2{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);

    % a_z
    gla_indi.ca.W_v(2,2) = 1e-12;
    simout3{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);

    % eta1
    gla_indi.ca.W_v(1,1) = 1e-9;
    gla_indi.ca.W_v(2,2) = 1;
    simout4{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);

    % eta1, eta2
    gla_indi.ca.W_v(3,3) = 1e-3;
    simout5{i} = simGust(gust_grad_dist,time,is_gla_enabled,is_failure);
    
end

set_param(model,"FastRestart","off");
toc;

%% Plot results
Legend = {'Open loop','$a_z$, $\eta_1$, $\eta_7$','$a_z$, $\eta_1$','$a_z$','$\eta_1$','$\eta_1$, $\eta_7$'};
ymax = 3.25;
ymin = -1;
for i = 1:2
    % Relative WRBM
    figure
    plotGustRespCntrlVar({simout_open_loop,simout1{i},simout2{i},simout3{i},simout4{i},simout5{i}},'WRBM',Legend);
    ylim([ymin,ymax])
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_wrbm_',num2str(i)]);
    end

    % Load factor
    figure
    plotGustRespCntrlVar({simout_open_loop,simout1{i},simout2{i},simout3{i},simout4{i},simout5{i}},'LoadFactor');
    ylim([ymin,ymax])
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_acc_',num2str(i)]);
    end
    
    % Relative deflection of 1st bending mode
    figure
    plotGustRespCntrlVar({simout_open_loop,simout1{i},simout2{i},simout3{i},simout4{i},simout5{i}},'eta1');
    ylim([ymin,ymax])
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_eta1_',num2str(i)]);
    end
    
    % Relative deflection of 2nd bending mode
    figure
    plotGustRespCntrlVar({simout_open_loop,simout1{i},simout2{i},simout3{i},simout4{i},simout5{i}},'eta7');
    ylim([ymin,ymax])
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_eta7_',num2str(i)]);
    end
    
    % Control inputs
    figure
    plotGustRespCntrlVar({simout1{i}},'u');
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_u_',num2str(i),'_1'],2);
    end
    figure
    plotGustRespCntrlVar({simout2{i}},'u');
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_u_',num2str(i),'_2'],2);
    end
    figure
    plotGustRespCntrlVar({simout3{i}},'u');
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_u_',num2str(i),'_3'],2);
    end
    figure
    plotGustRespCntrlVar({simout4{i}},'u');
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_u_',num2str(i),'_4'],2);
    end
    figure
    plotGustRespCntrlVar({simout5{i}},'u');
    if is_tikz_export_desired
        writeTikz(['gust_response_cntrl_var_u_',num2str(i),'_5'],2);
    end
end

%% Plot function
function [] = plotGustRespCntrlVar(simout_cell,var_name,Legend)
    is_legend = false;
    if nargin>2
        is_legend = true;
    end
    hold on
    ds = 5;
    for i = 1:length(simout_cell)
        switch var_name
            case 'WRBM'
                plot(simout_cell{i}.WBM.Time(1:ds:end),simout_cell{i}.WBM.Data(1:ds:end,5)/simout_cell{i}.WBM.Data(1,5));
                ylabel('Relative WRBM')
            case 'LoadFactor'
                plot(simout_cell{i}.acc.Time(1:ds:end),-simout_cell{i}.acc.Data(1:ds:end)/9.81+1);
                ylabel('Load factor')
            case 'eta1'
                plot(simout_cell{i}.eta.Time(1:ds:end),simout_cell{i}.eta.Data(1:ds:end,6+1)/simout_cell{i}.eta.Data(1,6+1));
                ylabel('Relative deflection')
            case 'eta7'
                plot(simout_cell{i}.eta.Time(1:ds:end),simout_cell{i}.eta.Data(1:ds:end,6+7)/simout_cell{i}.eta.Data(1,6+7));
                ylabel('Relative deflection')
        end
    end
    switch var_name
        case 'u'
            fig_temp = figure;
            h = plot(rand(2,3),rand(2,3));
            c = get(h,'Color');
            colors_default = reshape([c{:}],3,[]);
            colors = interp1([1,9,19]',colors_default(:,end:-1:1)',1:19)';
            close(fig_temp);
            for j = 1:size(simout_cell{i}.u.Data,2)/2
                h(j) = plot(simout_cell{1}.u.Time(1:ds:end),simout_cell{1}.u.Data(1:ds:end,j),'Color',colors(:,j));
            end
            ylim([-1,1])
            ylabel('Control inputs')
            legend([h(1),h(7),h(13),h(19)],'Strip 1','Strip 7','Strip 13','Strip 19')
    end
   
    xlabel('Time, s')
    grid on
    box on
    if is_legend
        legend(Legend{:},'interpreter','latex');
    end
end

%% Export figure to TikZ function
function [] = writeTikz(filename,num_cols)
    if nargin<2
        num_cols = 1;
    end
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}',['legend columns=',num2str(num_cols)]};
    filename = exportFilename([filename,'.tex']);
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

