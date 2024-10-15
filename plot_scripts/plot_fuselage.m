% add folders to path
is_tigl_installed = addPath();

%% Init aircraft
if is_tigl_installed
    [aircraft,structure] = aircraftSe2aCreate( 'flexible', true, 'unsteady', true, 'stall', false,'Mach', 0.77, 'Alt',6000, 'pchfilename','na_Se2A-MR-Ref-v4-twist_GFEM_MTOAa_S103_DMIG.pch','AdjustJigTwist',true );
else
    load('data/aircraft_structure.mat');
    wingSetCustomActuatorPath(aircraft.wing_main);
end

fuselage = aircraft.fuselage;

%% Plot
figure

hold on

plot(fuselage.geometry.border_pos(1,:),fuselage.geometry.border_pos(3,:),'k-')

for i = 1:fuselage.n_segments+1
     plot(fuselage.geometry.border_pos(1,i)*[1,1],fuselage.geometry.border_pos(3,i)+[-0.5,0.5]*fuselage.geometry.width(i),'k-');
     if i>1
         plot([fuselage.geometry.border_pos(1,i-1),fuselage.geometry.border_pos(1,i)],...
             [fuselage.geometry.border_pos(3,i-1)+0.5*fuselage.geometry.width(i-1),fuselage.geometry.border_pos(3,i)+0.5*fuselage.geometry.width(i)],'k-');
         plot([fuselage.geometry.border_pos(1,i-1),fuselage.geometry.border_pos(1,i)],...
             [fuselage.geometry.border_pos(3,i-1)-0.5*fuselage.geometry.width(i-1),fuselage.geometry.border_pos(3,i)-0.5*fuselage.geometry.width(i)],'k-');
          plot([fuselage.geometry.border_pos(1,i-1),fuselage.geometry.border_pos(1,i)],...
             [fuselage.geometry.border_pos(3,i-1)+0.5*fuselage.geometry.width_visc(i-1),fuselage.geometry.border_pos(3,i)+0.5*fuselage.geometry.width_visc(i)],'k--');
         plot([fuselage.geometry.border_pos(1,i-1),fuselage.geometry.border_pos(1,i)],...
             [fuselage.geometry.border_pos(3,i-1)-0.5*fuselage.geometry.width_visc(i-1),fuselage.geometry.border_pos(3,i)-0.5*fuselage.geometry.width_visc(i)],'k--');
     end
end

axis equal

set(gca,'XDir','reverse')
set(gca,'YDir','reverse')

