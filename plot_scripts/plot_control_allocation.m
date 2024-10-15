clear all;

is_tigl_installed = addPath();

is_tikz_export_desired = false;

%% Init aircraft
fp_spec.Altitude	= 6000; % m
fp_spec.EAS         = 177; % m/s
% fp_spec.Altitude	= 7650; % m
% fp_spec.EAS         = 147; % m/s

[Ma,~] = altEas2MaTas( fp_spec.Altitude, fp_spec.EAS );

if is_tigl_installed
    [aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'Mach', Ma, 'pchfilename', 'na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch', 'AdjustJigTwist', true );
else
    load('data/aircraft_structure.mat');
    wingSetCustomActuatorPath(aircraft.wing_main);
end

%% Init controller
gla_indi = glaIndiCreate( aircraft, fp_spec, ...
    'ModeControlIdx', [1,7], 'WeightModes', [1,1e-12] );

%% Plot
figure
plotControlAllocation1(aircraft,gla_indi,fp_spec.Altitude,Ma,0)

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('control_allocation_1.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%%

gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));

figure
plotControlAllocation1(aircraft,gla_indi,fp_spec.Altitude,Ma,1)

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('control_allocation_3.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%%

gla_indi = glaIndiCreate( aircraft, fp_spec, ...
    'ModeControlIdx', [1,7], 'WeightModes', [1,1e-12] );

figure
plotControlAllocation2(aircraft,gla_indi,fp_spec.Altitude,Ma,0)

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('control_allocation_2.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%%

gla_indi.ca.W_u = eye(length(gla_indi.ca.W_u));

figure
plotControlAllocation2(aircraft,gla_indi,fp_spec.Altitude,Ma,1)

%% Export figure to TikZ
if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('control_allocation_4.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%%

function plotControlAllocation1(aircraft,gla_indi,h,Ma,varargin)

if isempty(varargin)
    is_paper = 0;
else
    is_paper = varargin{1};
end

atm = isAtmosphere(h);
V = Ma * atm.a;
B = indiCeLadVar(gla_indi.ce,atm.rho,V,Ma);

gla_indi.ca.W_v(1,1) = 1;
gla_indi.ca.W_v(2,2) = 1e-12;
gla_indi.ca.W_v(3,3) = 1e-12;


y = aircraft.wing_main.geometry.ctrl_pt.pos(2,aircraft.wing_main.geometry.segments.flap_depth>0) / aircraft.wing_main.params.b*2;

h=plot(zeros(4,4),zeros(4,4));
hold on
grid on

nu = zeros(length(gla_indi.ca.W_v),1);
nu(1) = 1;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h1=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(1).Color);
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(1).Color)

gla_indi.ca.W_v(2,2) = 1;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h2=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(2).Color);
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(2).Color)

gla_indi.ca.W_v(2,2) = 1e-12;
gla_indi.ca.W_v(3,3) = 1;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h3=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(3).Color);
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(3).Color)

gla_indi.ca.W_v(2,2) = 1;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h4=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(4).Color);
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(4).Color)

if is_paper
    gla_indi.ca.W_v(1,1) = 0.0025;
    gla_indi.ca.W_v(2,2) = 1;
    gla_indi.ca.W_v(3,3) = 1e-12;
    gla_indi.ca.gamma = 100;
    gla_indi.ca.W_u = diag(ones(1,length(gla_indi.ca.W_u)));
    Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
    h5=plot(y(1:end/2),Delta_u(1:end/2),'k--');
    plot(y(end/2+1:end),Delta_u(end/2+1:end),'k--')
end

box on
xlim([-1,1])
ylim([-0.08,0.125])
xlabel('Dimensionless span')
ylabel('Actuator command increment')
legend([h1,h2,h3,h4],'$a_z$ only','$\ddot{\eta}_1=0$','$\ddot{\eta}_7=0$','$\ddot{\eta}_1=\ddot{\eta}_7=0$','location','north','interpreter','latex')

end


function plotControlAllocation2(aircraft,gla_indi,h,Ma,varargin)

if isempty(varargin)
    is_paper = 0;
else
    is_paper = varargin{1};
end

atm = isAtmosphere(h);
V = Ma * atm.a;
B = indiCeLadVar(gla_indi.ce,atm.rho,V,Ma);

h=plot(zeros(4,4),zeros(4,4));
hold on
grid on

y = aircraft.wing_main.geometry.ctrl_pt.pos(2,aircraft.wing_main.geometry.segments.flap_depth>0) / aircraft.wing_main.params.b*2;

nu = zeros(length(gla_indi.ca.W_v),1);
nu(2) = 1;

gla_indi.ca.W_v(1,1) = 1e-12;
gla_indi.ca.W_v(2,2) = 1;
gla_indi.ca.W_v(3,3) = 1e-12;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h1=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(1).Color);
hold on
grid on
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(1).Color)

gla_indi.ca.W_v(1,1) = 1;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h2=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(2).Color);
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(2).Color)

gla_indi.ca.W_v(1,1) = 1e-12;
gla_indi.ca.W_v(3,3) = 1;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h3=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(3).Color);
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(3).Color)

gla_indi.ca.W_v(1,1) = 1;
Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
h4=plot(y(1:end/2),Delta_u(1:end/2),'Color',h(4).Color);
plot(y(end/2+1:end),Delta_u(end/2+1:end),'Color',h(4).Color)

if is_paper
    gla_indi.ca.W_v(1,1) = 0.0025;
    gla_indi.ca.W_v(2,2) = 1;
    gla_indi.ca.W_v(3,3) = 1e-12;
    gla_indi.ca.gamma = 100;
    gla_indi.ca.W_u = diag(ones(1,length(gla_indi.ca.W_u)));
    Delta_u = caIndiWls(gla_indi.ca,B,nu,zeros(1,38)');
    h5=plot(y(1:end/2),Delta_u(1:end/2),'k--');
    plot(y(end/2+1:end),Delta_u(end/2+1:end),'k--')
end

box on
xlim([-1,1])
ylim([-2.5,5]*1e-3)
xlabel('Dimensionless span')
ylabel('Actuator command increment')
legend([h1,h2,h3,h4],'$\ddot{\eta}_1$ only','$a_z=0$','$\ddot{\eta}_7=0$','$\ddot{\eta}_7=a_z=0$','location','north','interpreter','latex')

end

