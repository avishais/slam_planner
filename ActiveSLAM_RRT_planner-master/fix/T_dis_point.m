function [upper_bound,lower_bound]=T_dis_point(num_obs,std,mu)

t_value=tinv(0.975,num_obs-1);
lower_bound=mu-t_value.*std./sqrt(num_obs);
upper_bound=mu+t_value.*std./sqrt(num_obs);
diff=upper_bound-lower_bound;
[~,loc]=find(diff<deg2rad(30));
lower_bound(loc)=mu(loc)-deg2rad(15);
upper_bound(loc)=mu(loc)+deg2rad(15);

end