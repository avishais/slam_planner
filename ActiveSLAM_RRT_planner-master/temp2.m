figure(3)
clf
plot(q1(1),q1(2),'og','markerfacecolor','g','markersize',10)
hold on
plot(q2(1),q2(2),'or','markerfacecolor','r','markersize',10)
grid on
axis equal

quiver(q1(1),q1(2),cos(h1),sin(h1),0.2);
quiver(q2(1),q2(2),cos(h2),sin(h2),0.2);

plot(qc1(1),qc1(2),'xg','markerfacecolor','g','markersize',10)
plot(qc2(1),qc2(2),'xr','markerfacecolor','r','markersize',10)

th = 0:0.05:2*pi;
plot(r*cos(th)+qc1(1),r*sin(th)+qc1(2),'--g');
plot(r*cos(th)+qc2(1),r*sin(th)+qc2(2),'--r');

plot([q1(1) q2(1)],[q1(2) q2(2)],':k');
plot([qc1(1) qc2(1)],[qc1(2) qc2(2)],':k');

plot(qt1(1),qt1(2),'sg','markerfacecolor','g','markersize',10)
plot(qt2(1),qt2(2),'sr','markerfacecolor','r','markersize',10)

%%
figure(2)
clf
hold on
plot(q.config(1,min_ind),q.config(1,min_ind),'og','markerfacecolor','g','markersize',10)
hold on
plot(pt(1),pt(2),'or','markerfacecolor','r','markersize',10)
grid on
axis equal
plot(finish(1),finish(2),'ob','markerfacecolor','b','markersize',10)

quiver(q.config(1,min_ind),q.config(1,min_ind),cos(q.config(3,min_ind)),sin(q.config(3,min_ind)),0.2);
quiver(pt(1),pt(2),cos(pt(3)),sin(pt(3)),0.2);
quiver(finish(1),finish(2),cos(finish(3)),sin(finish(3)),0.2);

plot(Q(1,:),Q(2,:),'-k');