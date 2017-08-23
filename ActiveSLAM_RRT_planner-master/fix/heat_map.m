%% initial all points in 3d
close all
clear
plot_fit_map;

load('corner_info.mat')
kinect_param;

%create the map with the obstacles

obst.x=[];
obst.y=[];
obst.r=ones(400*length(walls)/2,1)*0.01;
for i=1:length(walls)/2
    eval(['wall',num2str(i),'_x=linspace(walls(2*i-1,1),walls(2*i,1),400)'';'])
    eval(['wall',num2str(i),'_y=linspace(walls(2*i-1,2),walls(2*i,2),400)'';'])
    eval(['obst.x=[obst.x;wall',num2str(i),'_x];']);
    eval(['obst.y=[obst.y;wall',num2str(i),'_y];']);
end
obst.x=obst.x+2.854;
obst.y=obst.y+0.9569;

figure;
hold on
axis equal
f = viscircles([obst.x,obst.y],obst.r,'color','black');
plot(Corner_w(1,:),Corner_w(2,:),'.k');
clear wall*
%% form t distribution
[upper_bound,lower_bound]=T_dis_point(Obs_corner,std_corner,Mean_corner);

%% initial discrete 3d grid
num_angle=30;
grid_size=0.1;


% x+2.854 y+0.9569
% section 1: -4<=x<0, -1.49<y<0
x_grid=-4:grid_size:3.2;
x_grid(end)=[];
y_grid=-6:grid_size:1.3;
xy_info=zeros(length(x_grid)*length(y_grid),2);
%test=zeros(length(x_grid)*length(y_grid),1);
for i=1:length(x_grid)
    for j=1:length(y_grid)
        xy_info((i-1)*length(y_grid)+j,1)=x_grid(i);
        xy_info((i-1)*length(y_grid)+j,2)=y_grid(j);
    end
end

[loc,~]=find(xy_info(:,1)<=0 & xy_info(:,2)>=0);
xy_info(loc,:)=[];

[loc,~]=find(xy_info(:,1)>=1.53 & xy_info(:,2)>=0.165);
xy_info(loc,:)=[];

[loc,~]=find(xy_info(:,1)<=1.69 & xy_info(:,2)<=-1.49);
xy_info(loc,:)=[];

clear *grid
clear loc
clear i j

xy_info(:,1)=xy_info(:,1)+2.854;
xy_info(:,2)=xy_info(:,2)+0.9569;

%plot(xy_info(:,1),xy_info(:,2),'.r');

angle_info=linspace(-pi,pi,num_angle+1);
angle_info(end)=[];

[num_point,~]=size(xy_info);

%% calculate num obs
obs_stat=zeros(num_point,num_angle);
flag=0;
if_plot=0;

for i=1:num_point
    for j=1:num_angle
        [n_visible, idx] = isInFrustum_pts(xy_info(i,1), xy_info(i,2), angle_info(j),Corner_s,kinect,    upper_bound,lower_bound );
        obs_stat(i,j)= n_visible;
        
        if if_plot==1
            if flag==1
                children = get(gca, 'children');
                delete(children(1:2));
                flag=0;
            end
            if n_visible~=0
                quiver(xy_info(i,1),xy_info(i,2),0.2*cos(angle_info(j)),0.2*sin(angle_info(j)),'r');
                plot(Corner_w(1,idx),Corner_w(2,idx),'*r');
                flag=1;
                pause(0.01);
            end
        end
        
    end
end


%% plot the whole configuration space
% assume x is height, y is rho, and theta

alter=max(xy_info(:,1));


all_angle=sort(repmat(angle_info',num_point,1));
all_config=[repmat(xy_info,num_angle,1),all_angle];

clear all_angle

figure(2)
hold on
axis off

[x,y,z]=pol2cart(all_config(:,3),-all_config(:,1)+alter,all_config(:,2));
plot3(x,y,z,'.')
axis equal




%% plot configuration space meet threthold

thre=10;
[loc_xy,loc_angle]=find(obs_stat>=thre);
vis_config=[xy_info(loc_xy,:),angle_info(loc_angle)'];

robot_radius = 0.3;
idx = [];
for i = 1:size(vis_config)
    flag = true;
    for j = 1:length(obst.x)
        if norm([obst.x(j) obst.y(j)]-vis_config(i,1:2)) < robot_radius+obst.r(j)
            flag = false;
            break;
        end
    end
    if flag
        idx = [idx; i];
    end
end


% figure(4)
% hold on
% axis off
%
% [x,y,z]=pol2cart(vis_config(:,3),-vis_config(:,1)+alter,vis_config(:,2));
% plot3(x,y,z,'.r')
% axis equal

%%
figure(5);
clf
axis equal
f = viscircles([obst.x,obst.y],obst.r,'color','black');
hold on
plot3(vis_config(idx,1),vis_config(idx,2),vis_config(idx,3),'.r');
plot3(key_frame_w(:,1),key_frame_w(:,2),key_frame_w(:,4),'*k');
hold off
hold on



%%
rho=0.5;








