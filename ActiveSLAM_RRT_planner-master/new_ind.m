function file_ind = new_ind(path)
file_ind = sprintf('%s%04d.jpg',path,length(dir(sprintf('%s*.jpg',path)))+1);
end