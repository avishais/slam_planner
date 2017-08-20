%% set up

set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot,'DefaultAxesFontsize',24);
set(groot,'DefaultTextFontname','Times New Roman');
set(groot,'DefaultAxesFontname','Times New Roman');

close all
clear 
load('Corner_cloud_point_data.mat');

% draw walls
figure
hold on
axis equal

%%
for i=1:7
    plot(walls(2*i-1:2*i,1),walls(2*i-1:2*i,2),'k');
end

%% draw points cloud 
plot(Map_2D(:,1),Map_2D(:,2),'.','MarkerSize',1);





