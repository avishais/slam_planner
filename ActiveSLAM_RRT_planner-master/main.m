plot_fit_map;
%close all
%clear

load('corner_info.mat')
kinect_param;


dbstop if error;
%create the map with the obstacles

obst.x=[];
obst.y=[];
obst.r=ones(400*length(walls)/2,1)*0.05;
for i=1:length(walls)/2
    eval(['wall',num2str(i),'_x=linspace(walls(2*i-1,1),walls(2*i,1),400)'';'])
    eval(['wall',num2str(i),'_y=linspace(walls(2*i-1,2),walls(2*i,2),400)'';'])
    eval(['obst.x=[obst.x;wall',num2str(i),'_x];']);
    eval(['obst.y=[obst.y;wall',num2str(i),'_y];']);
end
obst.x=obst.x+2.854;
obst.y=obst.y+0.9569;

figure;
hold on
axis equal
f = viscircles([obst.x,obst.y],obst.r,'color','black');
plot3(Corner_w(1,:),Corner_w(2,:),Corner_w(3,:),'.k')

%%
%%define parameters
%define robot properties
rob_r = 0.4;
rob_turn = 0.25;

%create start and finish points 
start = [0; 0; 0];
finish = [2.93+2.854-0.2; -4+0.9569+1; -pi/2];
near_finish = 0.5;


plot(start(1),start(2),'gx','linewidth',3)
plot(finish(1),finish(2),'rx','linewidth',3)

%define collision checker parameters
tstep = 0.1;
path_step = 1;

%define save location 
dir = 'figures/';
save_images = false;

if save_images
    imwrite(print('-RGBImage'),new_ind(dir))
end

%%
%define q to hold all feasible configurations
q = struct();
q.config = start;
q.dist = 0;
q.parent = 0;
path = cell(1);
path_final = [];
%create a new random x,y configurations
iter = 10000;
config_test_x = (6.4+1)*rand([1,iter])-1;    %400 is hardcoded limit of the world
config_test_y = (4.85+2)*rand([1,iter])-4.85;
config_test=[config_test_x;config_test_y];

%parameter for rrt*
neighbour = 1; % search for the nodes in the neighbour

tic;
%start iterations
for i = 1 :iter
    q_len = length(q.parent);
    %display no. of iteration
    fprintf('iter.#%d:%d current time:%.2f\n',i,iter,toc);
    
    %create a random point
    pt = [config_test(:,i); atan2((finish(2)-config_test(2,i)),(finish(1)-config_test(1,i))) ];    %theta is pointing to finish direction (need better sampling method for theta)
    
    
    
    %check if the point is not in collision with obstacles
    col = InCollision_Node(pt,rob_r,obst); 
    %check how many point the camera can see
    
    if ~col
        %find the nearest point in q
        distance = dist(pt(1:2),q.config(1:2,:));
        [min_v, min_ind] = min(distance);
        
        if(min_v < 0.2)
            continue;
        end
        
        % adjust the angle to ensure a not too big turn
        alpha = -q.config(3,min_ind) + atan2((pt(2) - q.config(2,min_ind)),((pt(1) - q.config(1,min_ind))));
        if (alpha>2*pi)
            alpha = alpha - 2*pi;
        elseif (alpha<-2*pi)
            alpha = alpha + 2*pi;
        end
        
        if (alpha > pi/2)
            theta_0 = (alpha - pi) + q.config(3,min_ind);
        elseif (alpha < -pi/2)
            theta_0 = (alpha + pi) + q.config(3,min_ind);
        else
            theta_0 = alpha + q.config(3,min_ind);
        end
        
        beta = deg2rad(-15:5:15);
        n_visible = zeros(size(beta));
        for idx_beta = 1:length(beta)
            theta = beta(idx_beta) + theta_0;
            [n_visible(idx_beta),~] = isInFrustum_pts(pt(1),pt(2),theta,Corner_s,Norm_corner,kinect,Obs_corner,std_corner,Mean_corner);
        end
        [v,max_idx_beta] = max(n_visible);
        pt(3) = beta(max_idx_beta) + theta_0;
        
        % steer the robot toward the point
        if(min_v > path_step)
           
            pt(1:2) = q.config(1:2,min_ind) + path_step/min_v*(pt(1:2) - q.config(1:2,min_ind));
            alpha = -q.config(3,min_ind) + atan2((pt(2) - q.config(2,min_ind)),((pt(1) - q.config(1,min_ind))));
            if (alpha>2*pi)
                alpha = alpha - 2*pi;
            elseif (alpha<-2*pi)
                alpha = alpha + 2*pi;
            end

            if (alpha > pi/2)
                theta_0 = (alpha - pi) + q.config(3,min_ind);
            elseif (alpha < -pi/2)
                theta_0 = (alpha + pi) + q.config(3,min_ind);
            else
                theta_0 = alpha + q.config(3,min_ind);
            end  
            
            beta = deg2rad(-15:5:15);
            n_visible = zeros(size(beta));
            for idx_beta = 1:length(beta)
                theta = beta(idx_beta) + theta_0;
                [n_visible(idx_beta),~] = isInFrustum_pts(pt(1),pt(2),theta,Corner_s,Norm_corner,kinect,Obs_corner,std_corner,Mean_corner);
            end
            [v,max_idx_beta] = max(n_visible);
            pt(3) = beta(max_idx_beta) + theta_0;
            
        end
        
        %create path from point pt to nearest point in q given by index min_ind
        %check that the path is collision free
        [col, Q, T] = testpath(pt,q.config(:,min_ind),rob_turn,tstep,obst,Corner_s,Norm_corner,kinect,Obs_corner,std_corner,Mean_corner, false);
        
        
        if ~col
            %add parameters of the new pt in struct q
            q.config(:,q_len+1) = pt;
            q.parent(q_len+1) = min_ind;
            q.dist(q_len+1) = T(end);   %dist is the duration of the path
            path{q_len+1} = Q;          %path is added into cell array for better query
            
              % RRT star
%             %find the n nodes in the neighbour
%             %for all of them compute the cost, which is the distance
%             %traveled, get the smallest cost and add to the graph
%             idx_neighbour = find(distance < neighbour & distance > min_v);
%             if(~isempty(idx_neighbour))
%                
%                 for idx_nn = 1:length(idx_neighbour)
%                    
%                     [col, Q, T] = testpath(pt,q.config(:,idx_neighbour(idx_nn)),rob_turn,tstep,obst,Corner_s,Norm_corner,kinect,Obs_corner,std_corner,Mean_corner);
%                     if ~col
%                         if(q.dist(q_len+1) > T(end))
%                            
%                             q.parent(q_len+1) = idx_neighbour(idx_nn);
%                             q.dist(q_len+1) = T(end);
%                             path{q_len+1} = Q;
%                             disp('update')
%                             
%                         end
%                         
%                     end
%                     
%                 end
%                 
%             end
            
            
            %plot the new segment
            figure(1)
            if flag==1
                children = get(gca, 'children');
                delete(children(1));
                flag=0;
            end
            plot(pt(1),pt(2),'xb');
            line(Q(1,:),Q(2,:));
            [~,idx]=isInFrustum_pts(Q(1,1),Q(2,1),Q(3,1),Corner_s,Norm_corner,kinect,Obs_corner,std_corner,Mean_corner);
            plot3(Corner_w(1,idx),Corner_w(2,idx),Corner_w(3,idx),'*r');
            flag=1;
            drawnow;
            %pause(2)
            
            if save_images
                imwrite(print('-RGBImage'),new_ind(dir))
            end
            %check if the point is near the end point according to the predifined threshold
            if dist(finish(1:2),pt(1:2)) <= near_finish
                
                [col, Q, T] = testpath(pt,finish,rob_turn,tstep,obst,Corner_s,Norm_corner,kinect,Obs_corner,std_corner,Mean_corner,true);
                if ~col     
                %this step is required as distance between the created point and the end point might be too close to connect
                    q.config(:,q_len+2) = finish;
                    q.parent(q_len+2) = q_len+1;
                    path{q_len+2} = Q;
                    q.dist(q_len+2) = T(end);
                    
                    %connect to the end point
                    line(Q(1,:),Q(2,:));
                    if save_images
                        imwrite(print('-RGBImage'),new_ind(dir))
                    end
                    %backward indexing to get from end to start
                    [ind] = find_path(q);
                    final_time = 0;
                    %draw the final path and calculate the trip time
                    for k = 1 : length(ind)-2
                        final_time = final_time+q.dist(ind(k));
                        plot(path{ind(k)}(1,:),path{ind(k)}(2,:),'r','linewidth',4);
                        path_final = [path_final, [path{ind(k)}(1,:);path{ind(k)}(2,:);path{ind(k)}(3,:)]];
                        if save_images
                            imwrite(print('-RGBImage'),new_ind(dir))
                        end
                    end
                    
                    fprintf('Congratulations, a path was found! Trip time = %.2f units\n',final_time);
                    path_final = fliplr(path_final);
                    return;
                end
            end
        end
    end
       
        
end


%if the path is not found it is recommended to increase the number of iterations
display('No path was found! try increasing the iterations');