%AA Planar Kinematic Analysis
clc; clear variables;

%% Error Control Parameters
qtol=10^-6;         %Tolerance in solving for q
h=0.01;           %Time step

tfinal=2*pi;           %Final simulation time

%% Application Data
    %app=1, Slider-Crank
    %app=2, Quick Return
    %app=3, Windshield Wiper

app=1;


global ux uy zer
ux=[1;0];  %x-unit vector
uy=[0;1];  %y-unit vector
zer=zeros(2,1);  %2x1 zero vector

[nb,ngc,nh,nhc,nd,PJDT,q0e]=AppData(app);     %AppData(app)
par=[nb;ngc;nh;nhc;nd;qtol;app];   %Parameter vector for analysis control
%% Data Storage Arrays
Q=zeros(ngc,10);
Qd=zeros(ngc,10);
Qdd=zeros(ngc,10);

Q(:,1)=q0e;

%% Kinematic Analysis
n=1; % index n
t(1)=0; % Starting time

maxIter = 25; % Maximum iterations
while t(n)<tfinal && i - 1 < maxIter %Loop through line 126 && det(Phiq) ~= 0   
    n=n+1;
    t(n)=t(n-1)+h;
    tn=t(n);
    tnm=t(n-1);
    
    [Pf,Pfst,Pfstt]=P5Eval(tn,par);
    
    %Evaluate q
    
    q=Q(:,n-1);     %q-estimate
    if n>=1
        q=q+h*Qd(:,n-1);
    end
    
    i=1;    %Position iteration counter
    err=qtol+1;
    while err >qtol % ()&& det(Phiq) ~= 0 %Iteration for q, through line 54
        Phi=PhiEval(q,PJDT,par);
        Phiq=PhiqEval(q,PJDT,par);
        if det(Phiq) == 0
            fprintf('Warning! Jacobian matrix is non invertible since its determinant is zero. \n')
            break
        else
            delq=-Phiq\(Phi+Pf);  %Newton-Raphson iteration
            q=q+delq;
            err=norm(Phi+Pf);
            i=i+1;
        end
    end
    iter(n)=i-1;  %Report number of iterations
    Q(:,n)=q;  %Record q
    
    %Evaluate qd
    Phiq=PhiqEval(q,PJDT,par);
    if det(Phiq) == 0
        fprintf('Warning! Jacobian matrix is non invertible since its determinant is zero. \n')
        break
    else
        CondPhiq(n)=cond(Phiq);  %Record condition number of Jacobian
        if n > 2
            Condcheck = abs(CondPhiq(n)/CondPhiq(n-1));
            if Condcheck >= 100
                fprintf('Warning: Singularity-Large Condition Number Jump')
            end
        end
        qd=-Phiq\Pfst;  %Solution of velocity equation
        Qd(:,n)=qd;  % Record qd
        
        % Evaluate qdd
        P2=P2Eval(q,qd,PJDT,par);
        Gam=P2*qd+Pfstt;
        qdd=-Phiq\Gam;  %Solution of acceleration equation
        Qdd(:,n)=qdd;  %record qdd
        
        % Calculate output data of interest (Enter for each application)
        % For q,qd, & qdd, indices 1-3 refer to body 1, 4-6 refer to body 2, etc.
        % Index 1 of q is x positon, index 2 is y position, and 3 is phi.

        if app==1   %Slider-Crank
            phi1(n)=q(3);
            phi1d(n)=qd(3);
            phi1dd(n)=qdd(3);
            x2(n)=q(4);
            x2d(n)=qd(4);
            x2dd(n)=qdd(4);
        end
        
        if app==2   %Quick Return
            phi1(n)=q(3);
            phi2(n)=q(6);
            x4(n)=q(10);
            x4d(n)=qd(10);
            x4dd(n)=qdd(10);
        end
        
        if app==3   %Windshield Wiper
            phi1(n)=q(3);
            phi2(n)=q(6);
            phi2d(n)=qd(6);
            phi2dd(n)=qdd(6);
            phi3(n)=q(9);
            phi3d(n)=qd(9);
            phi3dd(n)=qdd(9);
            
        end
    end
end
%End analysis

%% Plots

% For example, to plot the x position of body 2 use plot(t,x2)

% figure
% plot()
% xlim()
% ylim()
% xlabel()
% ylabel()
