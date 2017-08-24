function UGV( q )

% q = [0; 0; 0*pi/4];

r = 0.25;
h = 0.3/2;
al = acos(h/(r));
w = r*sin(al);

M = [-w w w -w; 
    -h -h h h]';
C = [0.10 0.22 0.22 0.10;
    -0.06 -0.06 0.06 0.06]';

R = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))];

M = (R*M')' + repmat(q(1:2)',4,1);
C = (R*C')' + repmat(q(1:2)',4,1);
W1 = (R*wheel(-0.1,0.17)')' + repmat(q(1:2)',4,1);
W2 = (R*wheel(0.1,0.17)')' + repmat(q(1:2)',4,1);
W3 = (R*wheel(-0.1,-0.17)')' + repmat(q(1:2)',4,1);
W4 = (R*wheel(0.1,-0.17)')' + repmat(q(1:2)',4,1);

patch(M(:,1), M(:,2),'b');
patch(C(:,1), C(:,2),'r');
patch(W1(:,1), W1(:,2),'k');
patch(W2(:,1), W2(:,2),'k');
patch(W3(:,1), W3(:,2),'k');
patch(W4(:,1), W4(:,2),'k');

plot([0 r*cos(al)],[0 r*sin(al)],'k');

th = 0:0.1:2*pi;
plot(r*cos(th)+q(1), r*sin(th)+q(2), '-m');

end

function W = wheel(x,y)

W = [-0.05 0.05 0.05 -0.05;
    -0.04/2 -0.04/2 0.04/2 0.04/2]';

W = W + repmat([x y],4,1);

end