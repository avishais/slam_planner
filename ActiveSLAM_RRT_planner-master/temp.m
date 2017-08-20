function temp

q1 = [0.2; -0.1; 0.5];
q2 = [0.961094811725775;-0.276218686684658;-1.880581031756361];

r = 0.25;
tstep = 0.1;


[v,w,t] = GetShortestPath(q1,q2,r);

% find the path
Q=[q1];
T=[0];

for i=1:length(v)
    %tcur=linspace(tstep,t(i),1+ceil(t(i)/tstep)); % !!!!!***** tsteps?
    tcur=linspace(0,t(i),1+ceil(t(i)/tstep)); % !!!!!***** tsteps?
    T=[T(1:end) tcur+T(end)];
    Qadd = myprop(Q(:,end),v(i),w(i),tcur);
    Q=[Q(:,1:end) Qadd(:,2:end)];
    
    % Avishai addition
    figure(2)
    clf
    plot3(q1(1),q1(2),q1(3),'og','markerfacecolor','g','markersize',10)
    hold on
    plot3(q2(1),q2(2),q2(3),'or','markerfacecolor','r','markersize',10)
    plot3(Q(1,:),Q(2,:),Q(3,:),'.-b')
    quiver3(q1(1),q1(2),q1(3),cos(q1(3)),sin(q1(3)),0,0.1);
    quiver3(q2(1),q2(2),q2(3),cos(q2(3)),sin(q2(3)),0,0.1);
    quiver3(Q(1,:),Q(2,:),0*Q(3,:),cos(Q(3,:)),sin(Q(3,:)),0*Q(3,:),0.1);
    hold off
    axis equal
    grid on
    view(2)
    
end
Q'
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