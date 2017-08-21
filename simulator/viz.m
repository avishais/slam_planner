clear all
clc

obs = load('../data/obs.txt');

Q = load('../paths/path.txt');
Qms = load('../paths/path_milestones.txt');
% load('path1.mat');
% Q = path_final';

%%

vid = false;

if vid
    writerObj = VideoWriter('./videos/slam_rob1.avi');%,'MPEG-4');
    open(writerObj);
end

for i = 1:1:size(Q,1)
    
    figure(1)
    clf
    hold on
    axis equal
    %f = viscircles(obs(:,1:2),obs(:,3),'color','black');
    plot(obs(:,1),obs(:,2),'.k');
    
    plot(Q(:,1),Q(:,2),'.-k');
    plot(Qms(:,1),Qms(:,2),'or');
    
    plot(Qms(1,1),Qms(1,2), 'or','markerfacecolor','r');
    plot(Qms(end,1),Qms(end,2), 'og','markerfacecolor','g');
    
    UGV(Q(i,:)');
    
    hold off
    drawnow;
    
    if vid
        %set(h, 'Position', [1 1 ax])
        Im = getframe;
        %I.cdata = imcrop(I.cdata,[1 1 ax]);
        writeVideo(writerObj, Im);
    end
    
end

if vid
    close(writerObj);
end