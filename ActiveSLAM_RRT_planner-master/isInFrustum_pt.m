function visible = isInFrustum_pt(x_w, y_w, theta_w, pt_s, normal_s, cam_parameter)
% check if a point is in camera's frustum
% coordinate frames:
% w: world frame, [origin:starting point, x: forward, y: left, z: up], the ros frame
% b: body frame, [origin:com, x: forward, y: left, z: up]
% s: slam frame, [orgin:starting point, x: right, y: down, z: forward]
% c: camera frame, [orgin:focal point, x: right, y: down, z: forward]

% args:
% x_w: robot's x position in the world frame
% y_w: robot's y position in the world frame
% theta_w: robot's orientation w.r.t the world frame
% pt_s: point's position in the slam frame
% normal_s: point's normal direction in the slam frame
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
R_wb = eul2rotm([0,0,theta_w]);
T_wb = [R_wb, t_wb; zeros(1,3),1];

R_sw = [0, -1, 0;...
        0, 0, -1;...
        1, 0, 0];
t_sw = [0.1;0;-0.25];
T_sw = [R_sw, t_sw; zeros(1,3),1];

R_bc = [0,0,1;...
        -1,0,0;...
        0,-1,0];
t_bc = [0.25;-0.1;0];
T_bc = [R_bc, t_bc; zeros(1,3),1];

T_sc = T_sw*T_wb*T_bc;
T_cs = pinv(T_sc);

hpt_s = [pt_s;1];
hpt_c = T_cs*hpt_s;

pt_c_x = hpt_c(1);
pt_c_y = hpt_c(2);
pt_c_z = hpt_c(3);

if (pt_c_z <= 0.05)
    visible = false;
    return;
end

invZ = 1/pt_c_z;

u = cam_parameter.fx*pt_c_x*invZ + cam_parameter.cx;
v = cam_parameter.fy*pt_c_y*invZ + cam_parameter.cy;

if u < cam_parameter.minX || u > cam_parameter.maxX
    visible = false;
    return;
end

if v < cam_parameter.minY || v > cam_parameter.maxY
    visible = false;
    return;
end

distance = norm(hpt_c(1:3));
if distance < cam_parameter.min_dist || distance > cam_parameter.min_dist
    visible = false;
    return;
end

dir_s = -(T_sc(1:3,4) - pt_s)/distance;
if dir_s'*normal_s < cam_parameter.min_normal_difference
   
    visible = false;
    return;
    
end

visible = true;



end