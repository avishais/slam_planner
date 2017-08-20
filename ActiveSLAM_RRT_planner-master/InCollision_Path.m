function [col] = InCollision_Path(Q,r,obst,pts_s,normal_s,cam_parameter,Obs_corner,std_corner,Mean_corner)
%
if isempty(obst)
    col = 0;
    return
end
    col = 0;
    for i = 1 : length(Q(1,:))
        col = max(dist([Q(1,i);Q(2,i)],[obst.x,obst.y]') <= r+obst.r');
        if(~col)
        %[number_visible,~]=Count_Visible_pose(Q(1,i),Q(2,i),Q(3,i),Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7);
        [number_visible]=isInFrustum_pts(Q(1,i),Q(2,i),Q(3,i), pts_s, normal_s, cam_parameter,Obs_corner,std_corner,Mean_corner);
        
            if(number_visible<30)
                %pt
                %number_visible
                col_vis=true;
            else 
                col_vis=false;
            end
        else 
            col_vis=true;
        end
        if col_vis
            col=col_vis;
            return
        end
    end
end

