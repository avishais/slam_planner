%% initalize
% set(groot,'defaultAxesTickLabelInterpreter','latex');
% set(groot,'defaulttextinterpreter','latex');
% set(groot, 'defaultLegendInterpreter','latex');
% set(groot,'DefaultAxesFontsize',24);
% set(groot,'DefaultTextFontname','Times New Roman');
% set(groot,'DefaultAxesFontname','Times New Roman');


close all
clear
Map=importdata('Map.txt');
Traj_s=importdata('KeyFrameTrajectory.txt');
Traj_s=Traj_s(:,2:8);

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






%%  Alter data

[loc,~]=find(Map(:,end-2)>=1);

Map_denoised=Map(loc,:);
[loc,~]=find(Map_denoised(:,2)>-0);
Map_denoised(loc,:)=[];

Map_reorder=[Map_denoised(:,3),-Map_denoised(:,1),-Map_denoised(:,2)];
Map_Normal=[Map_denoised(:,6),-Map_denoised(:,4),-Map_denoised(:,5)];
Obser_num=Map_denoised(:,end-2)';
Mean_rad=Map_denoised(:,end-1)';
std_rad=Map_denoised(:,end)';


%% rotae about z
%Map_denoised=Map_reorder;
%theta=0.03525;
%rotm1=rotz(rad2deg(theta));
rotm1 =...
   [0.999378783079067  ,-0.035242700398838     ,              0;...
   0.035242700398838 ,  0.999378783079067     ,              0;...
                   0,                   0 ,  1.000000000000000];
Map_rotate1=(rotm1*Map_reorder')';
Norm_rot1=(rotm1*Map_Normal')';
Tm1=[[rotm1,[0;0;0]];[0,0,0,1]];

% rotate about y
%theta=0.0235286+0.0182115;
%rotm2=roty(rad2deg(theta));
rotm2 =...
   [0.999129008493071                   ,0,   0.041727980871985;...
                   0  , 1.000000000000000                   ,0;...
  -0.041727980871985                   ,0,   0.999129008493071];

Map_rotate2=(rotm2*Map_rotate1')';
Norm_rot2=(rotm2*Norm_rot1')';
Tm2=[[rotm2,[0;0;0]];[0,0,0,1]];

% rotate about x
%theta=0.0155815;
%rotm3=rotx(rad2deg(theta));
rotm3 =...
   [1.000000000000000 ,                  0,                   0;...
                   0  , 0.999878610884841 , -0.015580869520065;...
                   0  , 0.015580869520065 ,  0.999878610884841];
Map_rot=(rotm3*Map_rotate2')';
Norm_s=(R_sw*rotm3*Norm_rot2');
Tm3=[[rotm3,[0;0;0]];[0,0,0,1]];

%% convert key frame
[num_key,~]=size(Traj_s);

key_frame_w=zeros(num_key,4);
for i=1:num_key
    rotm=quat2rotm([Traj_s(i,end),Traj_s(i,end-3),Traj_s(i,end-2),Traj_s(i,end-1)]);
    Disp=[Traj_s(i,1);Traj_s(i,2);Traj_s(i,3)];
    T_sc=[[rotm,Disp];[0,0,0,1]];
    T_wc_pre=T_sw\T_sc;
    T_wc=Tm3*Tm2*Tm1*T_wc_pre;
    T_wb=T_wc/T_bc;
    eul=rotm2eul(T_wb(1:3,1:3));
    if(eul(1)>pi)
        eul(1)=eul(1)-pi*2;
    elseif(eul(1)<-pi)
        eul(1)=eul(1)+pi*2;
    end
        
    key_frame_w(i,:)=[T_wb(1:3,4)',eul(1)];

end

[loc,~]=find(key_frame_w(:,1)>=-1 & key_frame_w(:,2)>=-5);
key_frame_w=key_frame_w(loc,:);

%%


[number,~]=size(Map_rot);
Map_rot=[Map_rot,ones(number,1)]';


Map_s=T_sw*Map_rot;

Map_s=Map_s(1:3,:);

clear Map_r*
clear Norm_r*
clear rotm*
clear theta
clear loc
clear number

[~,loc]=find(Map_s(3,:)>0 & Map_s(1,:)<5);
Corner_s=[Map_s(:,loc);ones(1,length(loc))];
Norm_corner=Norm_s(:,loc);
Obs_corner=Obser_num(:,loc);
std_corner=std_rad(:,loc);
Mean_corner=Mean_rad(:,loc);


Corner_w=pinv(T_sw)*Corner_s;
Corner_s(end,:)=[];
clear loc
