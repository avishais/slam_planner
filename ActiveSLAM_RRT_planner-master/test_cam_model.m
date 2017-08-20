clear

close all   

load('Point_cloud_08_02.mat')



kinect_param;
x_w=1;
y_w=0;
theta_w=0;


[n_visible, idx_visible,hpt_c] = isInFrustum_pts(x_w, y_w, theta_w, Map_s, Norm_s, kinect);


figure(1)
axis equal
%axis xz
axis image

xlabel('x')
ylabel('y')
zlabel('z')

hold on

plot3(Map_s(1,:),Map_s(2,:),Map_s(3,:),'.k')
plot3(Map_s(1,idx_visible),Map_s(2,idx_visible),Map_s(3,idx_visible),'*r')
%plot3(hpt_c(1,:),hpt_c(2,:),hpt_c(3,:),'.')


%theta_w=pi/4;


%[n_visible, idx_visible,hpt_c] = isInFrustum_pts(x_w, y_w, theta_w, Map_s, Norm_s, kinect);

%plot3(Map_s(1,idx_visible),Map_s(2,idx_visible),Map_s(3,idx_visible),'*b')
%plot3(hpt_c(1,:),hpt_c(2,:),hpt_c(3,:),'.')