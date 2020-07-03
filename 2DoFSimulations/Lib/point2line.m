function [nearest_point, nearest_distance] = point2line(P0,P1,P2)
%%%%%%%%Calculate the distance from point to line
P0=reshape(P0,3,1);
P1=reshape(P1,3,1);
P2=reshape(P2,3,1);
P12 = P2-P1;
a=P12(1);
b=P12(2);
c=P12(3);
d=dot(P12,P0);
A=[a,b,c;b,-a,0;c,0,-a];
B=[d;b*P1(1)-a*P1(2);c*P1(1)-a*P1(3)];
%PP = A\B;
PP = A*pinv(B');
ND = norm(PP-P0);
%%%%%If the foot is within the endpoint, the foot is returned, otherwise the nearest endpoint is returned
if abs((norm(PP-P1)+norm(PP-P2))-norm(P1-P2))<0.001
    nearest_point=PP;
    nearest_distance=ND;
else
    D1 = norm(P0-P1);
    D2 = norm(P0-P2);
    if D1>D2
        nearest_point=P2;
        nearest_distance=D2;
    else
        nearest_point=P1;
        nearest_distance=D1;
    end
end

