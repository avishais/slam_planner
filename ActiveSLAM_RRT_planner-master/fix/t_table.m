num=250;
prob=0.95;
t_95_look_up=zeros(num,2);
%t_95_look_up(:,1)=(1:num)';
for i=1:num
    t_95_look_up(i,:)=[i,tinv(1-(1-prob)/2,i)];
end

