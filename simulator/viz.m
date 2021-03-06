clear all
clc

obs = load('../data/obs.txt');

Q = load('../paths/path.txt');
Qms = load('../paths/path_milestones.txt');

load('../data/data.mat');
load('../data/vis_config.mat');

%%

vid = false;

if vid
    writerObj = VideoWriter('./videos/slam_rob1.avi');%,'MPEG-4');
    open(writerObj);
end

cost = 0;
Cost_length = 0;
thres = 10;
min_n_visible = 1e9;
min_n_visible_heuristic = 1e9;
for i = 1:1:size(Q,1)
    
    figure(1)
    clf
    hold on
    axis equal
    %f = viscircles(obs(:,1:2),obs(:,3),'color','black');
    plot(obs(:,1),obs(:,2),'.k');
    
    plot3(vis_config(idx,1),vis_config(idx,2),vis_config(idx,3),'.r');
    
    plot3(Q(:,1),Q(:,2),Q(:,3),'.-k');
    plot3(Qms(:,1),Qms(:,2),Qms(:,3),'or');
    plot3(Qms(1,1),Qms(1,2),Qms(1,3), 'or','markerfacecolor','r');
    plot3(Qms(end,1),Qms(end,2),Qms(end,3), 'og','markerfacecolor','g');
    
    UGV(Q(i,:)');
    
    maxDistHeuristicValidity = 2;
    th = [0:0.1:2*pi 0];
    plot(maxDistHeuristicValidity*cos(th)+Qms(1,2), maxDistHeuristicValidity*sin(th)+Qms(1,2), '--k');
    
    % Compute number of features cost
    [n_visible, idex] = isInFrustum_pts(Q(i,1),Q(i,2),Q(i,3),Corner_s,kinect, upper_bound,lower_bound );
    plot(Corner_w(1,idex),Corner_w(2,idex),'*r');
    
    if n_visible < min_n_visible_heuristic && norm(Q(i,1:2)-Qms(1,1:2))<maxDistHeuristicValidity
        min_n_visible_heuristic = n_visible;
    end
    if n_visible < min_n_visible 
        min_n_visible = n_visible;
    end
    
    %disp(Q(i,:));
    %disp(n_visible);
    if i > 1 
        if n_visible < thres
            n_visible = 1e-4;
        end
        cost = cost + 1/n_visible;
    end
    
    % Compute length of path
    if i > 1
        Cost_length = Cost_length + norm(Q(i,:)-Q(i-1,:));
    end
    
    hold off
    axis([min(obs(:,1)) max(obs(:,1)) min(obs(:,2)) max(obs(:,2))]);
    grid on
    view(2)
    drawnow;
    
    if vid
        %set(h, 'Position', [1 1 ax])
        Im = getframe;
        %I.cdata = imcrop(I.cdata,[1 1 ax]);
        writeVideo(writerObj, Im);
    end
    
end

disp(['Cost of path: ' num2str(cost) ', ' num2str(1/cost) ]);
disp(['Cost length: ' num2str(Cost_length)]);
disp(['Min. visible: ' num2str(min_n_visible)]);
disp(['Min. visible heuristic: ' num2str(min_n_visible_heuristic)]);


if vid
    close(writerObj);
end