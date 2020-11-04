function [nb,ngc,nh,nhc,nd,PJDT,q0e]=AppData(app)

%Data in this beginning section does not change
%To modify data a particular system, enter data into the applicable "app"

%Unit Vectors for x & y dir
%ux=[1;0];       %can be used for when body is on x axix
%uy=[0;1];       %can be used for when body is on y axis
global ux uy zer

%Zero vector of length 2
%used to populate null fields in the Planar Joint Data Table
%zer=zeros(2,1);


%% 
if app==1   %Slider-Crank

nb=2;       %Number of bodies
ngc=3*nb;   %number of generalized coordinates
nh=3;       %Number of time independent holonomic constraints
nhc=5;      %Number of time independent holonomic constraint equations
nd=ngc-nhc; %Number of time dependent driving constraint equations


%Edit data below as needed for Planar Joint Data Table
%Driver constraint must be last joint
%*************************
%Joint 1, K=1  
%Rev-Body1 Bar to Ground
T1=1;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i1=1;           %i body conn.,i>0;
j1=0;           %j body conn.;
si1=-2*ux;      %vectors to Pi
sj1=zer;        %vectors to Pj
d1=0;           %dist.;
vi1=zer;        %i body vector
vj1=zer;        %j body vector

%Joint 2, K=2
%Rev-Body2 Crank to Ground
T2=1;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i2=2;           %i body conn.,i>0;
j2=0;           %body conn;
si2=zer;        %vectors to Pi
sj2=2*uy;       %vectors to Pj
d2=0;           %dist.;
vi2=zer;        %i body vector
vj2=zer;        %j body vector

%Joint 3, K=3
%Rev -Body2 crank to body 3 key
T3=1;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i3=2;           %bodies conn.,i>0;
j3=3;           %bodies conn.;
si3=1.5*ux;     %vectors to Pi
sj3=zer;        %vectors to Pj
d3=0;           %dist.;
vi3=zer;        %i body vector
vj3=zer;        %j body vector

%Joint 4, K=4 
%Tran-Body1 bar to body 3 key
T4=2;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i4=1;           %bodies conn.,i>0;
j4=3;           %bodies conn.;
si4=zer;        %vectors to Pi
sj4=zer;        %vectors to Pj
d4=0;           %dist.;
vi4=ux;         %i body vector
vj4=ux;         %j body vector

%Joint 5, K=5
%Tran -Body 4 cutter to ground
T5=2;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i5=4;           %bodies conn.,i>0;
j5=0;           %bodies conn.;
si5=zer;        %vectors to Pi
sj5=4*uy;        %vectors to Pj
d5=0;           %dist.;
vi5=ux;        %i body vector
vj5=ux;        %j body vector

%Joint 6, K=6
%Dist -Body1 bar to body 4 cutter
T6=3;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i6=1;           %bodies conn.,i>0;
j6=4;           %bodies conn.;
si6=2*ux;        %vectors to Pi
sj6=zer;        %vectors to Pj
d6=2.5298;           %dist.;
vi6=zer;        %i body vector
vj6=zer;        %j body vector

%Joint 7, K=7
%RelRotDrver-Body2 crank to ground
T4=4;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
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
PJDT(:,1)=[T1;i1;j1;si1;sj1;d1;vi1;vj1];     %Revolute-bar to ground
PJDT(:,2)=[T2;i2;j2;si2;sj2;d2;vi2;vj2];     %Revolute-crank to ground
PJDT(:,3)=[T3;i3;j3;si3;sj3;d3;vi3;vj3];     %Revolute-crank to key
PJDT(:,4)=[T4;i4;j4;si4;sj4;d4;vi4;vj4];     %Trans.-bar to key
PJDT(:,5)=[T5;i5;j5;si5;sj5;d5;vi5;vj5];     %Trans.-cutter to ground
PJDT(:,6)=[T6;i6;j6;si6;sj6;d6;vi6;vj6];     %Dist.-bar to cutter
PJDT(:,7)=[T7;i7;j7;si7;sj7;d7;vi7;vj7];     %RotD-crank to ground


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
nhc=8;      %Number of time independent holonomic constraint equations
nd=ngc-nhc; %Number of time dependent driving constraint equations


%Edit data below as needed for Planar Joint Data Table
%Driver constraint must be last joint
%*************************
%Joint 1, K=1  
%Rev-Body1 Bar to Ground
T1=1;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i1=1;           %i body conn.,i>0;
j1=0;           %j body conn.;
si1=zer;        %vectors to Pi
sj1=zer;        %vectors to Pj
d1=0;           %dist.;
vi1=zer;        %i body vector
vj1=zer;        %j body vector

%Joint 2, K=2
%Rev-Body2 Bar to Ground 
T2=1;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i2=2;           %i body conn.,i>0;
j2=0;           %body conn;
si2=zer;        %vectors to Pi
sj2=-70*ux;     %vectors to Pj
d2=0;           %dist.;
vi2=zer;        %i body vector
vj2=zer;        %j body vector

%Joint 3, K=3
%Rev -Body3 to Ground 
T3=1;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i3=3;           %bodies conn.,i>0;
j3=0;           %bodies conn.;
si3=zer;        %vectors to Pi
sj3=30*ux;      %vectors to Pj
d3=0;           %dist.;
vi3=zer;        %i body vector
vj3=zer;        %j body vector

%Joint 4, K=4 
%Dist -Body1 to Body2 
T4=3;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i4=1;           %bodies conn.,i>0;
j4=2;           %bodies conn.;
si4=-5*uy;      %vectors to Pi
sj4=-8*uy;      %vectors to Pj
d4=70;          %dist.;
vi4=zer;        %i body vector
vj4=zer;        %j body vector

%Joint 5, K=5 
%Dist -Body2 to Body3 
T5=3;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i5=2;           %bodies conn.,i>0;
j5=3;           %bodies conn.;
si5=5*uy;       %vectors to Pi
sj5=5*uy;       %vectors to Pj
d5=100;         %dist.;
vi5=zer;        %i body vector
vj5=zer;        %j body vector

%Joint 6, K=6
%RelRotDrver -Body1 to Ground
T6=4;           %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD)
i6=1;           %bodies conn.,i>0;
j6=0;           %bodies conn.;
si6=zer;        %vectors to Pi
sj6=zer;        %vectors to Pj
d6=0;           %dist.;
vi6=zer;        %i body vector
vj6=zer;        %j body vector
%*************************


%PJDT(12,nh): Planar Joint Data Table (First nh not time dependent)
%PJTd(:,k)=[T;i;j;sipr;sjpr;d;vipr;vjpr]; k=joint No., 
    %T=joint type(1=Rev,2=Tran,3=Dist, 4=RotD), i&j=bodies conn.,
    %si&jpr=vectors to Pi&j, d=dist., vi&jpr=vectors along trans axis
PJDT(:,1)=[T1;i1;j1;si1;sj1;d1;vi1;vj1];     %Rev-bod 1 to ground
PJDT(:,2)=[T2;i2;j2;si2;sj2;d2;vi2;vj2];     %Rev-bod 2 to ground
PJDT(:,3)=[T3;i3;j3;si3;sj3;d3;vi3;vj3];     %Rev-bod 3 to ground
PJDT(:,4)=[T4;i4;j4;si4;sj4;d4;vi4;vj4];     %Dist.-bod 1 to bod 2
PJDT(:,5)=[T5;i5;j5;si5;sj5;d5;vi5;vj5];     %Dist.-bod 2 to bod 3
PJDT(:,6)=[T6;i6;j6;si6;sj6;d6;vi6;vj6];     %RotD-bod 1 to ground


%**Initial Generalized Coordinate Estimate**
%   Create a vector for each body 
%   format for the vectors is [x coord, y coord, angle]
%   if body is on an axis, the unit vectors ux or uy can be utilized
%   q0e vector is the initialized coordinate estimate for the system 
q10=[0;0;0];            %Body1
q20=[-70;0;0];          %Body2
q30=[30;0;0];           %Body3
q0e=[q10;q20;q30];      %total system

end

%%

%Error Check number of bodies defined in the system
ijRows=PJDT(2:3,:);             %ijRows is subset of PJDT containing i and j values
[CountBod]=unique(ijRows);      %defines unique values for i and j
NumBod=numel(CountBod)-1;       %counts up unique i and j values (subtracts for ground "0")
if NumBod~=nb
    errorText=sprintf('Error in # of bodies defined \nCheck "nb" value and chosen body values (i&j) for each joint. \n# of unique values of i and j (not including ground) should match "nb"');
    error(errorText)
else
end

%Error Check number of degrees of freedom defined for the system
%The following will count number of holonomic constraints equations from joint types
%and compare to "nhc" value
RevSum=2*sum(PJDT(1,:)==1);         %Revolute Joint eliminates 2 deg of freedom
TranSum=2*sum(PJDT(1,:)==2);        %Translational Joint eliminates 2 deg of freedom
DistSum=1*sum(PJDT(1,:)==3);        %Distance constraint elimnates 1 deg of freedom
ConstSum=RevSum+TranSum+DistSum;    %Sum the # of holonomic constraints
if ConstSum~=nhc
    errortext=sprintf('Error in defined degrees of freedom.\nCheck "nhc" value and values for T in PJDT table');
    error(errortext)
else
end

end
