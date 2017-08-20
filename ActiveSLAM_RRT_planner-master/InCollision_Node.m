function col = InCollision_Node(q,r,obst)

%q is configuration [3x1] array for x,y and theta
%r is scalar for robot radius
%obst is struct with fileds x,y&r

if isempty(obst)
    col = 0;
    return
end
col = max(dist(q(1:2),[obst.x,obst.y]') <= r+obst.r');

if col
    return
end

end

