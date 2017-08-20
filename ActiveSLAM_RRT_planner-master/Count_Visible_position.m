function [number_visible_set]=Count_Visible_position(x,y,number_pose,Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7)
%% [number_visible]=Count_Visible_feature(x,y,number of pose from 0 to 2pi ,Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7)

pose_set=linspace(0,2*pi,number_pose)';
number_visible_set=[rad2deg(pose_set),zeros(length(pose_set),1)];

visible_wall=Get_Visible_Wall(x,y,Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7);
displacement=visible_wall-[x,y];
Orientation=mod(atan2(displacement(:,2),displacement(:,1)),2*pi);

for i=1:length(pose_set)
    [number_visible,~]=Count_Visible_pose(x,y,pose_set(i),visible_wall,Orientation);
    number_visible_set(i,2)=number_visible; 
end
end