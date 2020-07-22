function frdyn = frdyn01(X,X0,V,lama,beta)
% 此函数用于计算动态的斥力场合斥力
% X为机器人的当前位置，X0为障碍物的位置，V为机器人的速度，
%lama为势场系数，beta是常数通常取2
X=reshape(X,3,1);
X0=reshape(X0,3,1);
V=reshape(V,3,1);
x=X(1);
y=X(2);
z=X(3);
x0=X0(1);
y0=X0(2);
z0=X0(3);
vx=V(1);
vy=V(2);
vz=V(3);
ctheta=dot((X-X0),V)/(norm(X-X0)*norm(V));
theta=acos(ctheta);
if(theta<=pi&&theta>pi/2)
frdyn=[ (lama*abs(x - x0)*sign(x - x0)*(abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(-(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)))^beta)/(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(3/2) + (beta*lama*(conj(vx)/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)) - (abs(x - x0)*sign(x - x0)*(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0)))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(3/2)))*(abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(-(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)))^(beta - 1))/(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2), (lama*abs(y - y0)*sign(y - y0)*(abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(-(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)))^beta)/(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(3/2) + (beta*lama*(conj(vy)/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)) - (abs(y - y0)*sign(y - y0)*(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0)))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(3/2)))*(abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(-(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)))^(beta - 1))/(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2), (beta*lama*(conj(vz)/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)) - (abs(z - z0)*sign(z - z0)*(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0)))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(3/2)))*(abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(-(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)))^(beta - 1))/(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2) + (lama*abs(z - z0)*sign(z - z0)*(abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(-(conj(vx)*(x - x0) + conj(vy)*(y - y0) + conj(vz)*(z - z0))/((abs(vx)^2 + abs(vy)^2 + abs(vz)^2)^(1/2)*(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(1/2)))^beta)/(abs(x - x0)^2 + abs(y - y0)^2 + abs(z - z0)^2)^(3/2)];
else
frdyn=[0 0 0];
end
end

