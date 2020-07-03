function selection_force=selection_force2D(repulse_vector,theta)
%%%%%%%%%%%Computational decision-making force%%%%%%%%%%%%%%%%%%%%
fr = reshape(repulse_vector,3,1);
fy=norm(fr)*[0;1;0];
fw=cross(fr,fy);
fw=fw/norm(fw);
w= acos(dot(fr,fy)/(norm(fr)*norm(fy)));
rotate_vector = -w*fw;
R= rotationVectorToMatrix(rotate_vector);
x=theta;
y=0;
z=0;
fs2=[x;y;z];
selection_force =inv(R)* fs2;
end
