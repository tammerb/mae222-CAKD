% Newton Raphson Solution of k Equaitons Phi(q)=0 for k Variables q
% Single variable version using the diff function for finding the 1st derivative
% in the Taylor Expansion

function [x0, counter] = newtonRaphson(x, xtol, x0, F)
    % arguments
        
    % syms x;
    % xtol=0.001;  % Enter error tolerance in satisfying F(x)=0
    % x0= 1000;   % Enter initial solution estimate
    % F=x+x^2-2;   %Enter function of F(x)
    Fx=diff(F,x);  %Take Deribative of F(x) to obatain Phiq(x)
    counter=0;   %Counter for Number of iteration
    err=xtol+1;  %Set initial error greater than error tolerance

    while err > xtol  %Iteration for x, through line 15
    F_eval=vpa(subs(F,x,x0));  %Evaluate F(x) at x0
    Fx_eval=vpa(subs(Fx,x,x0));  %Evaluate Fx(x) at x0
    delx=-F_eval/Fx_eval;  %Newton-Raphson iteration
    x0=x0+delx;
    err=norm(F_eval);
    counter=counter+1;    
end
