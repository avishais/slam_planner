function [col, Q, T] = testpath(q1,q2,r,tstep,obst,pts_s,normal_s,cam_parameter,Obs_corner,std_corner,Mean_corner,isfinal)
% Get shortest path between these two configurations q1 and q2
if isfinal == false
    [v,w,t] = GetShortestPath(q1,q2,r);
else
    q2(3) = q1(3);
    [v,w,t] = GetShortestPath(q1,q2,r);
end

%check if path is feasible
if (isempty(v))
    display('path is infeasible!');
    col = 1;
    Q=[];
    T=[];
    return
end

% find the path
Q=[q1];
T=[0];
col = 0;

for i=1:length(v)
    tcur=linspace(tstep,t(i),1+ceil(t(i)/tstep));
    %tcur=linspace(0,t(i),1+ceil(t(i)/tstep));
    T=[T(1:end) tcur+T(end)];
    Q=[Q(:,1:end) myprop(Q(:,end),v(i),w(i),tcur)];
    
    % Avishai addition
%     figure(2)
%     clf
%     plot(q1(1),q1(2),'xg')
%     hold on
%     plot(q2(1),q2(2),'xr')
%     plot(Q(1,:),Q(2,:),'.-b')
%     quiver(q1(1),q1(2),cos(q1(3)),sin(q1(3)),0.1);
%     quiver(q2(1),q2(2),cos(q2(3)),sin(q2(3)),0.1);
%     hold off
%     axis equal
%     grid on
    
    [col] = InCollision_Path(Q,r,obst,pts_s,normal_s,cam_parameter,Obs_corner,std_corner,Mean_corner);
    if col
        return
    end
end



function q=myprop(q,v,w,t)
h=q(3)+w*t;
if (w~=0)
    x=q(1)+((v/w)*(sin(h)-sin(q(3))));
    y=q(2)-((v/w)*(cos(h)-cos(q(3))));
else
    x=q(1)+t.*v.*cos(h);
    y=q(2)+t.*v.*sin(h);
end
q=[x;y;h];

end

end