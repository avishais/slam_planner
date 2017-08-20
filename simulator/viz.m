clear all
clc

obs = load('../data/obs.txt');

Q = load('../paths/path.txt');
Qms = load('../paths/path_milestones.txt');
% load('path1.mat');
% Q = path_final';

%%

for i = 1:size(Q,1)

figure(1)
clf
hold on
axis equal
f = viscircles(obs(:,1:2),obs(:,3),'color','black');

plot(Q(:,1),Q(:,2),'.-k');
plot(Qms(:,1),Qms(:,2),'or');

plot(Qms(1,1),Qms(1,2), 'or','markerfacecolor','r');
plot(Qms(end,1),Qms(end,2), 'og','markerfacecolor','g');

UGV(Q(i,:)');

hold off
drawnow;
end