clear all
clc

obs = load('../data/obs.txt');

Q = load('../paths/path.txt');
Qms = load('../paths/path_milestones.txt');

load('../data/data.mat');

%%

vid = false;

if vid
    writerObj = VideoWriter('./videos/slam_rob1.avi');%,'MPEG-4');
    open(writerObj);
end

cost = 0;
Cost_length = 0;
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
    
    % Compute number of features cost
    [n_visible,idx]=isInFrustum_pts(Q(i,1),Q(i,2),Q(i,3),Corner_s,Norm_corner,kinect,Obs_corner,std_corner,Mean_corner);
    plot(Corner_w(1,idx),Corner_w(2,idx),'*r');
    disp(n_visible);
    if n_visible < 10 
        C = 1e-5;
    else
        C = n_visible;
    end
    cost = cost + 1/C;
    
    % Compute length of path
    if i > 1
        Cost_length = Cost_length + norm(Q(i,:)-Q(i-1,:));
    end
    
    hold off
    axis([min(obs(:,1)) max(obs(:,1)) min(obs(:,2)) max(obs(:,2))]);
    drawnow;
    
    if vid
        %set(h, 'Position', [1 1 ax])
        Im = getframe;
        %I.cdata = imcrop(I.cdata,[1 1 ax]);
        writeVideo(writerObj, Im);
    end
    
end

disp(['Cost of path: ' num2str(cost)]);
disp(['Cost length: ' num2str(Cost_length)]);

if vid
    close(writerObj);
end