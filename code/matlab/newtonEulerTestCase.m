load('proj2EvalCases.mat');

testCase = 56;

%--- Delete all previously created 'slBus#' object
busNum = 1;
deletingBuses = true;
while deletingBuses
    if (exist(['slBus',busNum]))
        clear(['slBus',busNum]);
        busNum = busNum + 1;
    else
        deletingBuses = false;
    end
end
% 'slBus1' should be 'linkList'
% 'slBus2' should be 'boundary_conditions'

global linkList;
global boundary_conditions;

linkList1 = struct('a',2.1352,'d',2.6184,'alpha',2.7520,'theta',[0],'mass',6.4479,'inertia',[1.7928,0,0;0,0.9645,0;0,0,0.0282],'com',[-0.3603;-0.1061;0.4806],'isRotary',1);
linkList2 = struct('a',0.6933,'d',[0],'alpha',0.1724,'theta',1.4137,'mass',2.4417,'inertia',[0.8579,0,0;0,0.0204,0;0,0,1.2176],'com',[0.1074;0.0884;-0.0666],'isRotary',0);
linkList3 = struct('a',0.2863,'d',0.1068,'alpha',2.4268,'theta',[0],'mass',1.4050,'inertia',[1.7597,0,0;0,0.1908,0;0,0,0.7051],'com',[-0.2531;-0.4911;0.3149],'isRotary',1);
linkList4 = struct('a',1.7555,'d',[0],'alpha',1.0536,'theta',0.9301,'mass',2.4179,'inertia',[1.3009,0,0;0,1.7147,0;0,0,0.1687],'com',[-0.0666;-0.3602;0.2519],'isRotary',0);
linkList5 = struct('a',0.0944,'d',2.5062,'alpha',2.1093,'theta',[0],'mass',3.2147,'inertia',[1.6129,0,0;0,1.2028,0;0,0,1.5792],'com',[-0.4501;0.0459;0.4432],'isRotary',1);
linkList6 = struct('a',0.1487,'d',0.8496,'alpha',0.9642,'theta',[0],'mass',5.6784,'inertia',[0.5979,0,0;0,0.5122,0;0,0,1.7731],'com',[-0.0103;0.4729;0.2485],'isRotary',1);
linkList = [linkList1,linkList2,linkList3,linkList4,linkList5,linkList6];
busInfo = Simulink.Bus.createObject(linkList);
%linkList = test_case(testCase).Inputs.I1;
paramList = test_case(testCase).Inputs.I2;
paramListDot = test_case(testCase).Inputs.I3;
paramListDDot = test_case(testCase).Inputs.I4;
boundary_conditions = test_case(testCase).Inputs.I5;
busInfo2 = Simulink.Bus.createObject(boundary_conditions);