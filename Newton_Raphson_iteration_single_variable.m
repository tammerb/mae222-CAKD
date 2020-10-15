% Newton Raphson Solution of k Equaitons Phi(q)=0 for k Variables q

syms q;
qtol=0.001;  % Enter error tolerance in satisfying Phi(q)=0
q0= 1000;   % Enter initial solution estimate
Phi=q+q^2-2;   %Enter function of Phi(q)
Phiq=diff(Phi,q);  %Take Deribative of Phi(q) to obatain Phiq(q)
counter=0;   %Counter for Number of iteration
err=qtol+1;  %Set initial error greater than error tolerance

while err >qtol  %Iteration for q, through line 15
Phieval=vpa(subs(Phi,q,q0));  %Evaluate Phi(q) at q0
Phiqeval=vpa(subs(Phiq,q,q0));  %Evaluate Phiq(q) at q0
delq=-Phieval/Phiqeval;  %Newton-Raphson iteration
q0=q0+delq;
err=norm(Phieval);
counter=counter+1;    
end
qresult=q0    %Solution
iter=counter  %Report number of iteations