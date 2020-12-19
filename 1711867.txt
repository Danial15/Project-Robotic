%% Assign d,a,alpha and theta
%Mohamad Danial Husaini bin Noordin (1711867)
% a1=15, a2=15, d3=20
L1 = Link('d',0,'a',15,'alpha',0);
L2 = Link('d',0,'a',15,'alpha',0); 
L3 = Link([0,20,0,0,1],'standard'); 
L3.qlim = [0 20]; %to limit the travel distance of link 3
L4 = Link('d',0,'a',0,'alpha',0); 
Rob = SerialLink ([L1 L2 L3 L4],'name','SCARAROBOT');
Rob.plot([0 0 0 0],'workspace',[-60 60 -60 60 -60 60]);
Rob.teach

%% take matrix value from excel file 1711867.xlsx
inputdata = readmatrix("1711867.xlsx", "sheet", "fkine_scara", "Range", "A2:E7");

%% transformation
q0 = [0 0 0 0];%initial position x=30 ,y=0, z=0
T1 = transl(inputdata(1,1),inputdata(1,2),inputdata(1,3));
T2 = transl(inputdata(2,1),inputdata(2,2),inputdata(2,3));
T3 = transl(inputdata(3,1),inputdata(3,2),inputdata(3,3));
T4 = transl(inputdata(4,1),inputdata(4,2),inputdata(4,3));
T5 = transl(inputdata(5,1),inputdata(5,2),inputdata(5,3));
T6 = transl(inputdata(6,1),inputdata(6,2),inputdata(6,3));

%% inverse kinematics
q1 = Rob.ikine(T1,[0 0 0 0],[1,1,1,0,0,0]);
q2 = Rob.ikine(T2,[0 0 0 0],[1,1,1,0,0,0]);
q3 = Rob.ikine(T3,[0 0 0 0],[1,1,1,0,0,0]);
q4 = Rob.ikine(T4,[0 0 0 0],[1,1,1,0,0,0]);
q5 = Rob.ikine(T5,[0 0 0 0],[1,1,1,0,0,0]);
q6 = Rob.ikine(T6,[0 0 0 0],[1,1,1,0,0,0]);

%% forward kinematics 
q1_T1 = Rob.fkine(q1);
q2_T2 = Rob.fkine(q2);
q3_T3 = Rob.fkine(q3);
q4_T4 = Rob.fkine(q4);
q5_T5 = Rob.fkine(q5);
q6_T6 = Rob.fkine(q6);

%% simulation of the robot  
t = [0: .5:2]';

% Path 1 = q0-->q1 
path1 = jtraj(q0,q1,t);
Rob.plot(path1,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);

% Path 2 = q1-->q2 
path2 = jtraj(q1,q2,t);
Rob.plot(path2,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);

% Path 3 = q2-->q3 
path3 = jtraj(q2,q3,t);
Rob.plot(path3,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);

% Path 4 = q3-->q4 
path4 = jtraj(q3,q4,t);
Rob.plot(path4,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);

% Path 5 = q4-->q5 
path5 = jtraj(q4,q5,t);
Rob.plot(path5,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);

% Path 6 = q5-->q6 
path6 = jtraj(q5,q6,t);
Rob.plot(path6,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);

% Path 7 = q6-->q7
path7 = jtraj(q6,q1,t);
Rob.plot(path7,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);

% Path 8 = q1-->q0
%we can see that the path will go back to q0. So, it is a close loop path.
path8 = jtraj(q1,q0,t);
Rob.plot(path8,'workspace',[-60 60 -60 60 -60 60]);
pause(0.02);
