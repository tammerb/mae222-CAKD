clc 
clear all
%Initial Conditions
Q0= [1;1;1];                %initial values
maxIter = 1000;             %set max # of iterations
qtol=1e-6;                  %set desired tolerance

%%Newton Raphson Iteration
Q=Q0;
Qold=Q0;

for i=1:maxIter             %Loop calls for NewtRaph fn in other file
    [f,j]=NewtRaph(Q);
    Q=Q-inv(j)*f;
    err(:,i)=abs(Q-Qold);
    Qold=Q;
    if (err(:,i)<qtol)      %checks if results are within error tolerance
        break;
    end
end

Q                           %display Q Matrix