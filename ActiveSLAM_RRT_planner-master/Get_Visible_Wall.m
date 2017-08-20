function [visible_wall]=Get_Visible_Wall(x,y,Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7)
%% [Num_Visible]=Get_Num_Visible_featuer[x(meter),y(meter),Wall_1,Wall_2,Wall_3,Wall_4,Wall_5,Wall_6,Wall_7]


%{
 wall1
    -4.0000         0
         0         0
 wall2
         0         0
         0    1.3000
 wall3
    1.9190         0
    1.9190    1.3000

 wall4
    1.9190         0
    3.6600         0

 wall5
   -4.0000   -1.4630
    2.2000   -1.4630

 wall6
    2.2000   -6.0000
    2.2000   -1.4630

 wall7
    3.6600         0
    3.6600   -6.0000
%}


%{
corner=[     0   ,      0;...
        1.9190   ,      0;...
        2.2000  , -1.4000];
%}

%% all cases
%-4<x<0 && -1.463<y<0 visible Wall_1 Wall_4 Wall_5 part of Wall_3 & Wall_7
if((-4<=x && x<0) && (-1.463<y && y<0))
    
    visible_wall=[Wall_1;Wall_4;Wall_5];
    
    % find threthold of wall 3
    y_thre_Wall_3=(0-y)*((1.919-x)/(0-x))+y;
    [loc,~]=find(Wall_3(:,2)<=y_thre_Wall_3);
    if( isempty(loc)~=1)
        temp=Wall_3(loc,:);
        visible_wall=[visible_wall;temp];
    end
    
    % find threthold of wall 7
    y_thre_Wall_7=(-1.436-y)*((3.66-x)/(2.2-x))+y;
    [loc,~]=find(Wall_7(:,2)>=y_thre_Wall_7);
    if( isempty(loc)~=1)
        temp=Wall_7(loc,:);
        visible_wall=[visible_wall;temp];
    end
    
    
%0<x<1.919 && -1.463<y<0 visible Wall_1 Wall_2 Wall_3 Wall_4 Wall_5 part of  Wall_7    
elseif((0<=x && x<1.919) && (-1.463<y && y<0))  
    visible_wall=[Wall_1;Wall_2;Wall_3;Wall_4;Wall_5];
    
    y_thre_Wall_7=(-1.436-y)*((3.66-x)/(2.2-x))+y;
    [loc,~]=find(Wall_7(:,2)>=y_thre_Wall_7);
    if( isempty(loc)~=1)
        temp=Wall_7(loc,:);
        visible_wall=[visible_wall;temp];
    end


%1.919<x<2.2 && -1.463<y<0 visible Wall_1 Wall_4 Wall_5 part of  Wall2 Wall_7    
elseif((1.919<=x && x<2.2) && (-1.463<y && y<0))  
    visible_wall=[Wall_1;Wall_4;Wall_5];
    
    y_thre_Wall_7=(-1.436-y)*((3.66-x)/(2.2-x))+y;
    [loc,~]=find(Wall_7(:,2)>=y_thre_Wall_7);
    if( isempty(loc)~=1)
        temp=Wall_7(loc,:);
        visible_wall=[visible_wall;temp];
    end
    
    y_thre_Wall_2=(0-y)*(x/(x-1.919))+y;
    [loc,~]=find(Wall_2(:,2)<=y_thre_Wall_2);
    if( isempty(loc)~=1)
        temp=Wall_2(loc,:);
        visible_wall=[visible_wall;temp];
    end
    
%2.2<x<3.66 && -1.463<y<0 visible Wall_1 Wall_4 Wall_5 Wall_6 Wall_7 part of  Wall2    
elseif((2.2<=x && x<3.66) && (-1.463<y && y<0))  
    visible_wall=[Wall_1;Wall_4;Wall_5;Wall_6;Wall_7];
    
    y_thre_Wall_2=(0-y)*(x/(x-1.919))+y;
    [loc,~]=find(Wall_2(:,2)<=y_thre_Wall_2);
    if( isempty(loc)~=1)
        temp=Wall_2(loc,:);
        visible_wall=[visible_wall;temp];
    end    
    
    
% 0<x<1.919 & y>=0 visible wall_2 wall_3 part of wall_5 & wall_7
elseif((0<x && x<1.919) && (y>=0))
    
    visible_wall=[Wall_2; Wall_3];
    
    x_thre_Wall_5_1=x-x*(y+1.436)/y;
    x_thre_Wall_5_2=x+(1.919-x)*(y+1.436)/y;
    [loc,~]=find((Wall_5(:,1)>=x_thre_Wall_5_1) & (Wall_5(:,1)<=x_thre_Wall_5_2));
    if( isempty(loc)~=1)
        temp=Wall_5(loc,:);
        visible_wall=[visible_wall;temp];
    end   
    
    if(x_thre_Wall_5_2>2.2)
        y_thre_wall_7_1=y-(y*(3.66-x)/(1.919-x));
        y_thre_Wall_7_2=y-((y+1.436)*(3.66-x)/(2.2-x));
        [loc,~]=find((Wall_7(:,2)<=y_thre_wall_7_1) &( Wall_7(:,2)>=y_thre_Wall_7_2));
        if(isempty(loc)~=1)
            temp=Wall_7(loc,:);
            visible_wall=[visible_wall;temp];
        end
        
        
    end
    
    
% 2.2<x<3.66 & y<=-1.436 visible Wall_6 Wall_7 part of Wall_1 Wall_2 Wall_4    
elseif((2.2<x && x<3.66) && y<=-1.436)
    visible_wall=[Wall_6;Wall_7];

    x_thre_0=x-(x-2.2)*(0-y)/(-1.436-y);
    y_thre_Wall_2_1=y+(-y)*x/(x-2.2);
    y_thre_Wall_2_2=y+(-1.436-y)*x/(x-2.2);
    
    if(x_thre_0<=0)
        [loc,~]=find(Wall_1(:,1)>=x_thre_0);
        if(isempty(loc)~=1)
            temp=Wall_1(loc,:);
            visible_wall=[visible_wall;temp;Wall_4];
        end
        
        [loc,~]=find(Wall_2(:,2)<=y_thre_Wall_2_1);
        if(isempty(loc)~=1)
            temp=Wall_2(loc,:);
            visible_wall=[visible_wall;temp];
        end
        
        
    elseif(x_thre_0>0 && x_thre_0<1.919)
        [loc,~]=find(Wall_2(:,2)<=y_thre_Wall_2_1 & Wall_2(:,2)>=y_thre_Wall_2_2);
        if(isempty(loc)~=1)
            temp=Wall_2(loc,:);
            visible_wall=[visible_wall;temp;Wall_4];
        end   
    else
        [loc,~]=find(Wall_4(:,1)>=x_thre_0);
        if(isempty(loc)~=1)
            temp=Wall_4(loc,:);
            visible_wall=[visible_wall;temp];
        end
        
    end

else
    visible_wall=[];
% end of case    

end


end
