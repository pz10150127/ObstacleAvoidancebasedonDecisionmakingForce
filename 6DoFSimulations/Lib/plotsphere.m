function plotsphere(pos,r)
[x,y,z]=ellipsoid(pos(1),pos(2),pos(3),r,r,r);
surf(x,y,z);
axis equal;
end

