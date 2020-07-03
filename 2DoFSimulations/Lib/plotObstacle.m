function [] = plotObstacle( x,y,r )  
%%%%%%%%Draw obstacles
theta=0:0.02*pi:2*pi;  
Circle1=x+r*cos(theta);  
Circle2=y+r*sin(theta);   
%plot(Circle1,Circle2,'k--');
%hold on
%scatter(x,y);
fill(Circle1,Circle2,'g');
axis equal  
end  