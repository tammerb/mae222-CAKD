function [nb,ngc,nh,nhc,nd,PJDT,q0e]=AppData(app)


%Data in this beginning section does not change
%To modify data a particular system, enter data into the applicable "app"

%Unit Vectors for x & y dir
ux=[1;0];       %can be used for when body is on x axix
uy=[0;1];       %can be used for when body is on y axis
%global ux uy 

%Zero vector of length 2
%used to populate non-applicable fields in the Planar Joint Data Table
zer=zeros(2,1);

%% 
if app==1   %Slider-Crank

nb=2;       %Number of bodies
ngc=3*nb;   %number of generalized coordinates
nh=3;       %Number of time independent holonomic constraints
nhc=5;      %Number of time independent holonomic constraint equations
nd=ngc-nhc; %Number of time dependent driving constraint equations

%Edit data below as needed for Planar Joint Data Table
%*************************
%Joint 1, K=1 
%Rev-Body1 Crank to Ground
i1=1;           %i body conn.,i>0;
j1=0;           %j body conn.;
si1=zer;        %vectors to Pi
sj1=zer;        %vectors to Pj
d1=0;           %dist.;
vi1=zer;        %i body vector
vj1=zer;        %j body vector

%Joint 2, K=2
%Tran-Body2 Slider to Ground
i2=2;           %i body conn.,i>0;
j2=0;           %body conn;
si2=zer;        %vectors to Pi
sj2=zer;        %vectors to Pj
d2=0;           %dist.;
vi2=ux;         %i body vector
vj2=ux;         %j body vector


%Joint 3, K=3
%Dist.-Body1 Crank to Body2 slider
i3=1;           %bodies conn.,i>0;
j3=2;           %bodies conn.;
si3=ux;         %vectors to Pi
sj3=zer;        %vectors to Pj
d3=1.01;        %dist.;
vi3=zer;        %i body vector
vj3=zer;        %j body vector


%Joint 4, K=4
%RelRotDrver-Body1 crank to ground
i4=1;           %bodies conn.,i>0;
j4=0;           %bodies conn.;
si4=zer;        %vectors to Pi
sj4=zer;        %vectors to Pj
d4=0;           %dist.;
vi4=zer;        %i body vector
vj4=zer;        %j body vector
%*************************


%PJDT(12,nh): Planar Joint Data Table (First nh joints not time dependent)
%PJDT(:,k)=[T;i;j;sipr;sjpr;d;vipr;vjpr]; k=joint No., 
%T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD), i&j=bodies connected,
%sipr&sjpr=vectors to Pi&Pj in joint definition, d=dist., 
%vipr&vjpr=vectors along translstional axis
PJDT(:,1)=[1;i1;j1;si1;sj1;d1;vi1;vj1];%Revolute-crank to ground
PJDT(:,2)=[2;i2;j2;si2;sj2;d2;vi2;vj2];%Trans.-slider2 to ground
PJDT(:,3)=[3;i3;j3;si3;sj3;d3;vi3;vj3];%Dist.-crank to slider2
PJDT(:,4)=[4;i4;j4;si4;sj4;d4;vi4;vj4];%RelRotDrver-crank to ground


%**Initial Generalized Coordinate Estimate**
%   Create a vector for each body 
%   format for the vectors is [x coord, y coord, angle]
%   if body is on an axis, the unit vectors ux or uy can be utilized
%   q0e vector is the initialized coordinate estimate for the system 
q10=[0;0;0];        %body 1
q20=[2.2*ux;0];     %body 2
q0e=[q10;q20];      %total system
end
%% 
if app==2   %Quick Return

nb=4;       %Number of bodies
ngc=3*nb;   %number of generalized coordinates
nh=6;       %Number of time independent holonomic constraints
nhc=11;     %Number of time independent holonomic constraint equations
nd=ngc-nhc; %Number of time dependent driving constraint equations


%Edit data below as needed for Planar Joint Data Table
%*************************
%Joint 1, K=1  
%Rev-Body1 Bar to Ground
i1=1;           %i body conn.,i>0;
j1=0;           %j body conn.;
si1=-2*ux;      %vectors to Pi
sj1=zer;        %vectors to Pj
d1=0;           %dist.;
vi1=zer;        %i body vector
vj1=zer;        %j body vector

%Joint 2, K=2
%Rev-Body2 Crank to Ground 
i2=2;           %i body conn.,i>0;
j2=0;           %body conn;
si2=zer;        %vectors to Pi
sj2=2*uy;       %vectors to Pj
d2=0;           %dist.;
vi2=zer;        %i body vector
vj2=zer;        %j body vector


%Joint 3, K=3
%Rev -Body2 crank to body 3 key
i3=2;           %bodies conn.,i>0;
j3=3;           %bodies conn.;
si3=1.5*ux;     %vectors to Pi
sj3=zer;        %vectors to Pj
d3=0;           %dist.;
vi3=zer;        %i body vector
vj3=zer;        %j body vector


%Joint 4, K=4 
%Tran-Body1 bar to body 3 key 
i4=1;           %bodies conn.,i>0;
j4=3;           %bodies conn.;
si4=zer;        %vectors to Pi
sj4=zer;        %vectors to Pj
d4=0;           %dist.;
vi4=ux;         %i body vector
vj4=ux;         %j body vector

%Joint 5, K=5
%Tran -Body 4 cutter to ground
i5=4;           %bodies conn.,i>0;
j5=0;           %bodies conn.;
si5=zer;        %vectors to Pi
sj5=4*uy;        %vectors to Pj
d5=0;           %dist.;
vi5=ux;        %i body vector
vj5=ux;        %j body vector

%Joint 6, K=6
%Dist -Body1 bar to body 4 cutter
i6=1;           %bodies conn.,i>0;
j6=4;           %bodies conn.;
si6=2*ux;        %vectors to Pi
sj6=zer;        %vectors to Pj
d6=2.5298;           %dist.;
vi6=zer;        %i body vector
vj6=zer;        %j body vector

%Joint 7, K=7
%RelRotDrver-Body2 crank to ground
i7=2;           %bodies conn.,i>0;
j7=0;           %bodies conn.;
si7=zer;        %vectors to Pi
sj7=zer;        %vectors to Pj
d7=0;           %dist.;
vi7=zer;        %i body vector
vj7=zer;        %j body vector
%*************************


%PJDT(12,nh): Planar Joint Data Table (First nh not time dependent)
%PJTd(:,k)=[T;i;j;sipr;sjpr;d;vipr;vjpr]; k=joint No., 
    %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD), i&j=bodies conn.,
    %si&jpr=vectors to Pi&j, d=dist., vi&jpr=vectors along trans axis
PJDT(:,1)=[1;i1;j1;si1;sj1;d1;vi1;vj1];    %Revolute-bar to ground
PJDT(:,2)=[1;i2;j2;si2;sj2;d2;vi2;vj2];    %Revolute-crank to ground
PJDT(:,3)=[1;i3;j3;si3;sj3;d3;vi3;vj3];    %Revolute-crank to key
PJDT(:,4)=[2;i4;j4;si4;sj4;d4;vi4;vj4];      %Trans.-bar to key
PJDT(:,5)=[2;i5;j5;si5;sj5;d5;vi5;vj5];      %Trans.-cutter to ground
PJDT(:,6)=[3;i6;j6;si6;sj6;d6;vi6;vj6];     %Dist.-bar to cutter
PJDT(:,7)=[4;i7;j7;si7;sj7;d7;vi7;vj7];    %RotD-crank to ground


%**Initial Generalized Coordinate Estimate**
%   Create a vector for each body 
%   format for the vectors is [x coord, y coord, angle]
%   if body is on an axis, the unit vectors ux or uy can be utilized
%   q0e vector is the initialized coordinate estimate for the system 
q10=[1.2;1.6;0.9273];       %Body 1
q20=[0;2;0];                %Body 2
q30=[1.5;2;0.9273];         %Body 3
q40=[0;4;0];                %Body 4
q0e=[q10;q20;q30;q40];      %total system

end
%% 
if app==3   %Windshield Wiper

nb=3;       %Number of bodies
ngc=3*nb;   %number of generalized coordinates
nh=5;       %Number of time independent holonomic constraints
nhc=8;     %Number of time independent holonomic constraint equations
nd=ngc-nhc; %Number of time dependent driving constraint equations


%PJDT(12,nh): Planar Joint Data Table (First nh not time dependent)
%PJTd(:,k)=[T;i;j;sipr;sjpr;d;vipr;vjpr]; k=joint No., 
    %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD), i&j=bodies conn.,
    %si&jpr=vectors to Pi&j, d=dist., vi&jpr=vectors along trans axis
PJDT(:,1)=[1;1;0;zer;zer;0;zer;zer];    %Rev-bod 1 to ground
PJDT(:,2)=[1;2;0;zer;-70*ux;0;zer;zer]; %Rev-bod 2 to ground
PJDT(:,3)=[1;3;0;zer;30*ux;0;zer;zer];  %Rev-bod 3 to ground
PJDT(:,4)=[3;1;2;-5*uy;-8*uy;70;zer;zer]; %Dist.-bod 1 to bod 2
PJDT(:,5)=[3;2;3;5*uy;5*uy;100;zer;zer]; %Dist.-bod 2 to bod 3
PJDT(:,6)=[4;1;0;zer;zer;0;zer;zer];    %RotD-bod 1 to ground

%Initial generalized coordinate estimate
q10=[0;0;0];
q20=[-70;0;0];
q30=[30;0;0];
q0e=[q10;q20;q30];

end


end







