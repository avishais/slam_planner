function [n_visible, idx_visible] = isInFrustum_pts(x_w, y_w, theta_w, pts_s, normal_s, cam_parameter,num_obs,std,mu)
% check if a set of points is in camera's frustum
% coordinate frames:
% w: world frame, [origin:starting point, x: forward, y: left, z: up], the ros frame
% b: body frame, [origin:com, x: forward, y: left, z: up]
% s: slam frame, [orgin:starting point, x: right, y: down, z: forward]
% c: camera frame, [orgin:focal point, x: right, y: down, z: forward]

% args:
% x_w: robot's x position in the world frame (scalar)
% y_w: robot's y position in the world frame (scalar)
% theta_w: robot's orientation w.r.t the world frame (scalar)
% pts_s: point's position in the slam frame (3 by n matrix, n points)
% normal_s: point's normal direction in the slam frame (3 by n matrix, n points)
% cam_parameter: parameter of camera includes:
%               -fx: focal length in x
%               -fy: focal lenght in y
%               -cx: principle point in x
%               -cy: principle point in y
%               -maxX: maximum coordinate in x
%               -minX: minimum coordinate in x
%               -maxY: maximum coordinate in y
%               -minY: minimum coordinate in y
%               -max_dist: maximum visible distance 
%               -min_dist: minimum visible distance
%               -min_normal_difference: minimum difference in normal
%               direction

t_wb = [x_w; y_w; 0];
R_wb = eul2rotm([theta_w,0,0]);
T_wb = [R_wb, t_wb; zeros(1,3),1];

R_sw = [0, -1, 0;...
        0, 0, -1;...
        1, 0, 0];
t_sw = [-0.1;0;-0.25];
T_sw = [R_sw, t_sw; zeros(1,3),1];

R_bc = [0,0,1;...
        -1,0,0;...
        0,-1,0];
t_bc = [0.25;-0.1;0];
T_bc = [R_bc, t_bc; zeros(1,3),1];

T_sc = T_sw*T_wb*T_bc;
T_cs = pinv(T_sc);

n_pts = size(pts_s,2);
hpt_s = [pts_s;ones(1, n_pts)]; % homogeneous coordinates
hpt_c = T_cs*hpt_s;

pt_c_x = hpt_c(1,:);
pt_c_y = hpt_c(2,:);
pt_c_z = hpt_c(3,:);

idx_visible = [1:n_pts];
% step 1: rule out the points with depth smaller than 0.05
idx_positive_z = (pt_c_z>0.05);
idx_visible = idx_visible(idx_positive_z);

% step 2: rule out the points which are out of current view
invZ = 1./pt_c_z(idx_positive_z);
u = cam_parameter.fx*pt_c_x(idx_positive_z).*invZ + cam_parameter.cx;
v = cam_parameter.fy*pt_c_y(idx_positive_z).*invZ + cam_parameter.cy;
idx_uv = (u > cam_parameter.minX & u < cam_parameter.maxX)&(v > cam_parameter.minY & v < cam_parameter.maxY);
idx_visible = idx_visible(idx_uv);

% step 3: rule out the points which are too close or too far away
distance = sqrt(sum(abs(hpt_c(1:3,idx_visible)).^2,1));
idx_distance = (distance > cam_parameter.min_dist & distance < cam_parameter.max_dist);
idx_visible = idx_visible(idx_distance);

% step 4: rule out the points whose viewing direction is different from
% normal direction when building the map
if(isempty(idx_visible))
    idx_visible = 1;
    n_visible = 0;
    return;
end

dir_s = -(repmat(T_sc(1:3,4),1,length(idx_visible)) - pts_s(:,idx_visible))./repmat(distance(idx_distance),3,1);
theta_s=atan2(dir_s(1,:),dir_s(3,:));

%idx_dir = (dir_s'*normal_s(:,idx_visible) > cam_parameter.min_normal_difference);

%t_value=tinv(0.999,num_obs-1);
%lower_bound=mu(idx_visible)-t_value(idx_visible).*std(idx_visible)./sqrt(num_obs(idx_visible));
%upper_bound=mu(idx_visible)+t_value(idx_visible).*std(idx_visible)./sqrt(num_obs(idx_visible));

lower_bound = mu(idx_visible) - 3*std(idx_visible);
upper_bound = mu(idx_visible) + 3*std(idx_visible);

diff=upper_bound-lower_bound;
[~,loc]=find(diff<deg2rad(30));
mean_vis=mu(idx_visible);
lower_bound(loc)=mean_vis(loc)-deg2rad(15);
upper_bound(loc)=mean_vis(loc)+deg2rad(15);
% diff=upper_bound-lower_bound

idx_dir=(theta_s>lower_bound & theta_s<upper_bound);

idx_visible = idx_visible(idx_dir);



n_visible = length(idx_visible);

end