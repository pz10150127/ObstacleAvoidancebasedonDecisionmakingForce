function [dx,ddx]=diff_motion2(t,x)
dx=diff_motion(t,x);
ddx=diff_motion(t,dx);