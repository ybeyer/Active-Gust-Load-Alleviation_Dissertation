clear all;

addPath();

is_tikz_export_desired = false;

%% Optimum booster factor
b=linspace(1,8,100);
k = [0.5,0.6,0.7,0.8,0.9];

figure
hold on
grid on
box on
Legend={};
for i = 1:length(k)
    plot(b,(1-k(i))*sqrt(b)+k(i)./b)
    Legend{i} = ['$k=',num2str(k(i)),'$'];
end

ylim([0,2])

k=linspace(k(1),0.95,100);
b_opt=(2*k).^(2/3)./(k.^2-2*k+1).^(1/3);
T=(1-k).*sqrt(b_opt)+k./b_opt;
T(b_opt>b(end))=[];
b_opt(b_opt>b(end))=[];
plot(b_opt,T,'k--')
Legend{end+1} = 'Optimum';

xlim([min(b),max(b)]);
xlabel('Booster factor $b$','interpreter','latex')
ylabel('Time delay reduction $T/T_0$','interpreter','latex')
legend(Legend{:},'interpreter','latex','location','northwest')

%% Export figure to TikZ

if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}','legend columns=2'};
    filename = exportFilename('booster_avoid_noise_1.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end


%% Bode plot of sensor filter adjustment

T_act = 0.016;
b = 4;
omega_sens = 500;
d_sens = 1;

h_act = tf(1,[T_act,1]);
h_sens = tf(1,[1/omega_sens^2,2*d_sens/omega_sens,1]);
h_boost = tf(b/T_act*[T_act,1],[1,b/T_act]);
omega_sens2 = omega_sens/sqrt(b);
h_sens2 = tf(1,[1/omega_sens2^2,2/omega_sens2,1]);


[mag1,phase1,wout1] = bode(h_sens);
[mag2,phase2,wout2] = bode(h_sens*h_boost,wout1);
[mag3,phase3,wout3] = bode(h_sens2*h_boost,wout1);

fig = figure;

subplot(2,1,1)
hold on
semilogx(wout1,20*log10(squeeze(mag1)));
semilogx(wout2,20*log10(squeeze(mag2)));
semilogx(wout3,20*log10(squeeze(mag3)));
set(gca,'XScale','log')
ylim([-inf,10])
grid on
box on
ylabel('Magnitude, dB')

legend('$H_{\mathrm{sens},0}$','$H_{\mathrm{sens},0} H_{\mathrm{boost}}$', '$H_{\mathrm{sens}} H_{\mathrm{boost}}$','interpreter','latex','location','southwest')


subplot(2,1,2)
hold on
semilogx(wout1,squeeze(phase1));
semilogx(wout2,squeeze(phase2));
semilogx(wout3,squeeze(phase3));
set(gca,'XScale','log')
grid on
box on
xlabel('Frequency, rad/s')
ylabel('Phase, deg')


%% Export figure to TikZ

if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('booster_avoid_noise_2.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end

%% Bode plot of original and boosted sensor-filter-actuator combination

[mag1,phase1,wout1] = bode(h_act*h_sens,wout1);
[mag2,phase2,wout2] = bode(h_act*h_boost*h_sens2,wout1);

figure

subplot(2,1,1)
hold on
semilogx(wout1,20*log10(squeeze(mag1)));
set(gca,'XScale','log')
ylim([-inf,10])
grid on
box on
ylabel('Magnitude, dB')

subplot(2,1,2)
hold on
semilogx(wout1,squeeze(phase1));
set(gca,'XScale','log')
grid on
box on
xlabel('Frequency, rad/s')
ylabel('Phase, deg')

subplot(2,1,1)
semilogx(wout2,20*log10(squeeze(mag2)));
legend('$H_{\mathrm{sens},0} H_{\mathrm{act}}$','$H_{\mathrm{boost}} H_{\mathrm{sens}} H_{\mathrm{act}}$','interpreter','latex','location','southwest')
subplot(2,1,2)
semilogx(wout2,squeeze(phase2));

%% Export figure to TikZ

if is_tikz_export_desired
    tikzwidth = '\figurewidth';
    tikzheight = '\figureheight';
    tikzfontsize = '\tikzstyle{every node}=[font=\tikzfontsize]';
    extra_axis_options = {'ylabel style={font=\tikzfontsize}','xlabel style={font=\tikzfontsize}','ticklabel style={/pgf/number format/fixed}','legend style={font=\tikzfontsize}'};
    filename = exportFilename('booster_avoid_noise_3.tex');
    matlab2tikz(filename,'width',tikzwidth,'height',tikzheight,'extraCode',tikzfontsize,'extraAxisOptions',extra_axis_options);
end
