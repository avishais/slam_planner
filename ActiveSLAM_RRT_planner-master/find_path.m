function [ind] = find_path(q)

cur_ind = length(q.parent);
ind = cur_ind;
    while true
        cur_ind = q.parent(cur_ind);
        ind = [ind(1:end), cur_ind];
        if cur_ind == 0
           return 
        end
    end

end