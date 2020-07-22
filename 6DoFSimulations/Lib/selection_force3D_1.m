function selection_force=selection_force3D_1(theta,repulse_vector)
fr = reshape(repulse_vector,3,1);
fz=norm(fr)*[0;0;1];
fw=cross(fr,fz);
fw=fw/norm(fw);
w= acos(dot(fr,fz)/(norm(fz)*norm(fz)));
if fw(3)>0.5*(fr(3)+fz(3)) %代表逆时针旋转，w为正
    rotate_vector = w*fw;
else                                 %代表顺时针旋转，w为负
    rotate_vector = -w*fw;
end
R= rotationVectorToMatrix(rotate_vector);
%theta=0;
x=cos(theta);
y=sin(theta);
z=0*x;
fs2=[x;y;z];
selection_force =inv(R)* fs2;
