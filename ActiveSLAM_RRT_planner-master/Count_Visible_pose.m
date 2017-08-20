function [number_visible,seen_points]=Count_Visible_pose(x,y,pose,Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7)
% option 1 [number_visible]=Count_Visible_pose(x,y,pose(rad),Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7)
% option 2 [number_visible]=Count_Visible_pose(x,y,pose(rad),Visible_wall)
% option 3 [number_visible]=Count_Visible_pose(x,y,pose(rad),Visible_wall,Orientation of each points with respect to the robot in world frame)
number_visible=0;
seen_points=[];

    switch nargin
        case 4
            visible_wall=Wall_1;
            displacement=visible_wall-repmat([x,y],size(visible_wall,1),1);
            Orientation=mod(atan2(displacement(:,2),displacement(:,1)),2*pi);
           
        case 10
             visible_wall=Get_Visible_Wall(x,y,Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7);
             if(isempty(visible_wall))
                number_visible=0;
                return;
             end
             displacement=visible_wall-repmat([x,y],size(visible_wall,1),1);
             Orientation=mod(atan2(displacement(:,2),displacement(:,1)),2*pi);
             
        case 5
            visible_wall=Wall_1;
            Orientation=Wall_2;

        otherwise
            return;
    end
    if(isempty(visible_wall))
        number_visible=0;
        
    else
        Low_angle_bound=mod(pose-pi/4,2*pi);
        High_angle_bound=mod(pose+pi/4,2*pi);

        if(Low_angle_bound<High_angle_bound)
            [loc,~]=find(Orientation<=High_angle_bound & Orientation>=Low_angle_bound);
        else
            [loc,~]=find(Orientation<=High_angle_bound|Orientation>=Low_angle_bound);
        end
        if(isempty(loc)~=1)
            seen_points=visible_wall(loc,:);
            number_visible=length(loc);
        else
            number_visible=0;
        end
    end




end