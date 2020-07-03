function dx=diff_motion(t,x)
n = length(x);
%same interval
if length(t)==1
    t=t*(0:(n-1));
end
x2=x(2:n);
x1=x(1:(n-1));
t2=t(2:n);
t1=t(1:(n-1));

dx=(x2-x1)./(t2-t1);
%Use spline interpolation to determine the last element
dx(n)=spline(t1,dx,t(n));
end

