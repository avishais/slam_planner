function y=mysinc(x)
% Returns sin(x)/x (MATLAB defines "sinc" slightly differently).
    y = sinc(x/pi);
end
