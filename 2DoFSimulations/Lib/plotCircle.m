function [] = plotCircle( x,y,r )  
theta=0:0.05*pi:2*pi;  
Circle1=x+r*cos(theta);  
Circle2=y+r*sin(theta);   
plot(Circle1,Circle2,'k-.');  
axis equal  
end  