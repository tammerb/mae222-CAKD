function [Pf,Pfst,Pfstt]=P5Eval(tn,par)

[nb,ngc,nh,nhc,nd,qtol,app]=parPart(par);

% Enter Constraint t derivatives of P(t,par);

%Check if the driver function is in general form omega*t
%For general case required user inputs are omega and set general = true
%For non-genral case required user inputs are driver function and
%corresponding first and second order derivative. set general false
true = 1;
false = 0;

if app==1   %Slider-Crank
omega=1;
general = true; %True for gernal case; Flase for non-general case
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if app==2   %Quick Return
omega=4;
general= true ; %True for gernal case; Flase for non-general case
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if app==3   %Windshield Wiper
omega=2;
general= true ; %True for gernal case; Flase for non-general case
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if app==4   %Planar Double Slider-Crank Assignment
omega=1;
general= false; %True for gernal case; Flase for non-general case
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if general == true
    PDf=[omega*tn];        %Driver functions of tn
    Pf=[zeros(nhc,1);PDf];  %Time dependent driver vector
    PDfd=[omega];       %First derivatives of driver functions of tn
    Pfst=[zeros(nhc,1);PDfd];  %Time dependent driver velocity vector
    PDfdd=[0];      %Second derivatives of driver functions of tn   
    Pfstt=[zeros(nhc,1);PDfdd];  %Time dependent driver acceleration vector
else
    %if the PDf is not the general case, such as PDf= omega*tn^2 for app 4
    omega=1;
    PDf=[omega*tn^2];        %Enter nd driver functions of tn
    Pf=[zeros(nhc,1);PDf];  %Time dependent driver vector
    PDfd=[2*omega*tn];       %Enter nd first derivatives of driver functions of tn
    Pfst=[zeros(nhc,1);PDfd];  %Time dependent driver velocity vector
    PDfdd=[2*omega];      %Enter nd second derivatives of driver functions of tn   
    Pfstt=[zeros(nhc,1);PDfdd];  %Time dependent driver acceleration vector
end 

end
