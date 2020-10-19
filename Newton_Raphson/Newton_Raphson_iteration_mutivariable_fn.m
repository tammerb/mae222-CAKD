% Newton Raphson Solution of n Equaitons F(x)=0 for n Variables x

% Clear Everything
clearvars;          % Clear all variables from the workspace
clc;                % Clear Command Window

%This case use a 3 dimension as an example (This code works for n-dimensions)

%define variable
syms x1 x2 x3;      % Define varible base on number of dimension (x1 ... xn)
sym_x = [x1 x2 x3].';% System of variable x

%Enter the parameters needed
xtol = 0.001;  % Enter error tolerance in satisfying F(x)=0
x0 = [1 2 3].';   % Enter initial solution estimate 1xn matrix
sym_F = [x1+x2^2-2 x1+x3 x1+x2].';   % Enter system of function of F(x)

%Run the Newton Raphason function and get the results
[counter, x0] = newtonRaphson(sym_x, xtol, x0, sym_F)

function [counter, x0] = newtonRaphson(sym_x, xtol, x0, sym_F)

H=jacobian(sym_F, sym_x);  %Take Deribative of F(x) to obatain Phiq(x)
counter=0;   %Counter for Number of iteration
err=xtol+1;  %Set initial error greater than error tolerance

while err >xtol  %some thing else Iteration for x, through line 15
F_eval=vpa(subs(sym_F,sym_x,x0));  %Evaluate F(x) at x0
Fx_eval=vpa(subs(H,sym_x,x0));  %Evaluate Fx(x) at x0
delx = inv(Fx_eval)*F_eval;  %Evaluate delta x

x0=x0-delx;  %Get a set of new values for iteration/solution
err=norm(F_eval); %Calculate the error using 2 norm
counter=counter+1;
end
end

