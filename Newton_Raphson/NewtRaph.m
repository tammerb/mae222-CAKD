function [phi,jac]=NewtRaph(Q)
syms x y z
%Define variables in terms of Q matrix
x=Q(1); 
y=Q(2);
z=Q(3);

%Define functions
phi(1,1)=x^2-y;
phi(2,1)=2*x-y*x+z;
phi(3,1)=x*y+z^2;

%Jacobian Matrix
% jac=[2*x, -1, 0;
%     2-y, -x, 1;
%     y, x, 2*z]; 


%jac=jacobian(phi,[x, y, z]);
j=jacobian(phi, [x, y, z]);
jac=subs(j,{x,y,z},0);
