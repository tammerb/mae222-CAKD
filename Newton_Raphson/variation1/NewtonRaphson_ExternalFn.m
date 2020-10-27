%%Newton Raphson Solution Using External Function

%Clear command window and all stored variables
clear all;          
clc;               


%Define variables
syms q1 q2 q3;              % Define varible base on number of dimension (x1 ... xn)
Q = [q1 q2 q3].';           % Matrix of variables

qtol = 0.001;               % Enter error tolerance in satisfying Phi(q)=0
q0 = [0.1 0.1 0.1].';       % Matrix of initial solution estimates

i=1;                        %Iteration Counter
err=qtol+1;                 %Setting initial error for loop 

%Iterative loop to find q
while err > qtol           
        [sym_Phi, Phiq] = NewPhiEval(Q);    
        Phi_eval=vpa(subs(sym_Phi,Q,q0));   %Evaluates Phi(q) at q0
        Phiq_eval=vpa(subs(Phiq,Q,q0));     %Evaluate Jacobian of Phi(q) at q0
        delq = inv(Phiq_eval)*Phi_eval;         %Newton-Raphson Iteration
        q0=q0-delq;                             %Sets value of q0 for next iteration
        err=norm(Phi_eval);                     %Calculates error of current solution
        i=i+1;
end
print=q0
iter=i-1
