%%NEW PhiEval Function

%External function called by NewtonRaphson solver
%Use this to define system of equations, function will calculate Jacobian
%for the user

function [sym_Phi, Phiq] = NewPhiEval(Q)

sym_Phi = [2*Q(1,1)^2+Q(2,1)-Q(3,1) Q(3,1)^3-4 Q(2,1)^2+4*Q(2,1)-1].';  %Define system of equations of interest


Phiq=jacobian(sym_Phi, Q);                                              %Calculate jacobian matrix of Phiq(q) system of equations

end