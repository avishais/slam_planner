function d=dist(x,y)
% Returns the distance between two points in xy space
    d = sqrt((x(2,:)-y(2,:)).^2 + (x(1,:)-y(1,:)).^2);
end
