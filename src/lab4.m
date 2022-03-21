% RBE3001 - Laboratory 3
%{
Notes Go Here
=======================
Time to get to our points in the triangle
0>1:    0.5750
1>2:    0.7834
2>0:    0.7208
Startend
-71.7600
   63.5800
    4.9100

point1 =

   -0.9600
   51.1000
  -70.9300
    
point2 =

   76.0800
   70.3000
   -2.7700



%}
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

    robot = Robot(myHIDSimplePacketComs);
    traj_planner = Traj_Planner;
    disp "Started"
    L0 = 55;
    L1 = 40;
    L2 = 100;
    L3 = 100;
    homePointMM = [100; 0; 195];
    homePoint = [0 0 0];
%------------------Run Tests Here-------------------

%test_eStop(robot)
%constantCheckForSing(robot)
%testFDK(robot)
testJacob(robot);
%quintic_TS(robot);
%checkVelConstantly(robot)
%{
    startEnd = [95; -95; 33]
    point1 = [176; -13; 158]
    point2 = [151 102 49]
    these are in MM
    
%}


% targetPoint = [95 -95 33]
% robot.velMotionPlan(targetPoint)
%------------- -----Test Functions-----------------------  
robot.shutdown();

function constantCheckForSing(robot)
    while(true)
        singBool = robot.check_eStop();
    curr_pos = robot.measured_js(1, 0)
    robot.plot_armNODELAY(curr_pos(1,:));
    pause(0.1)
    end
end

function testFDK(robot)
     cp = robot.measured_js(1,1)
     e = fdk3001(robot,cp);
end

function testJacob(robot)
    %jacobTestPos = [0 0 -90];
    jacobTestPos = [0 90 -90];
    jp = robot.jacob3001(jacobTestPos)
    detJP = det(jp(1:3,:))
end

function test_eStop(robot)
    point = [0 90 -90]
    start = [0 80 20]
    robot.interpolate_jp(start,2000)
    pause(3)
    robot.interpolate_jp(point,2000)
    curr_pos = robot.measured_js(1, 0)
    data1 = robot.ik3001(curr_pos(1,:))
    target_diff = abs(curr_pos(1,:) - point)
    dlmwrite('eStopXYZ2.csv',data1,'delimiter',',');
    data = [0 0];
    dlmwrite('eStopDET2.csv',data,'delimiter',',');
    tic;
    while(any(target_diff>2))
        t = toc;
        curr_pos2 = robot.measured_js(1, 0);
        det = robot.check_eStop()
        data = [t det];
        point = robot.ik3001(curr_pos2(1,:))
        dlmwrite('eStopXYZ2.csv',transpose(point),'delimiter',',','-append');
        dlmwrite('eStopDET2.csv',data,'delimiter',',','-append');
    end
end

function pyramid(robot)
    startEnd = [80; -50; 95]
    startEndDEG = robot.ik3001(startEnd)
    robot.interpolate_jp(startEndDEG,2000)
    pause(3)
    top58 = [130; 0; 195]
    Lbottom19 = [80; 50; 95]
    Ltop26 = [180; 50; 95]
    Rtop37 = [180; -50; 95]
    Rbottom4 = [80; -50; 95]

    data1 = cubicForPyramid(Rbottom4,Lbottom19,robot);
    dlmwrite('BONUS_mov1.csv',data1,'delimiter',',','-append');
    data2 = cubicForPyramid(Lbottom19,Ltop26,robot);
    dlmwrite('BONUS_mov2.csv',data2,'delimiter',',','-append');
    data3 = cubicForPyramid(Ltop26,Rtop37,robot);
    dlmwrite('BONUS_mov3.csv',data3,'delimiter',',','-append');
    data4 = cubicForPyramid(Rtop37,Rbottom4,robot);
    dlmwrite('BONUS_mov4.csv',data4,'delimiter',',','-append');
    data5 = cubicForPyramid(Rbottom4,top58,robot);
    dlmwrite('BONUS_mov5.csv',data5,'delimiter',',','-append');
    data6 = cubicForPyramid(top58,Ltop26,robot);
    dlmwrite('BONUS_mov6.csv',data6,'delimiter',',','-append');
    data7 = cubicForPyramid(Ltop26,Rtop37,robot);
    dlmwrite('BONUS_mov7.csv',data7,'delimiter',',','-append');
    data8 = cubicForPyramid(Rtop37,top58,robot);
    dlmwrite('BONUS_mov8.csv',data8,'delimiter',',','-append');
    data9 = cubicForPyramid(top58,Lbottom19,robot)
    dlmwrite('BONUS_mov9.csv',data9,'delimiter',',','-append');
end

function data = cubicForPyramid(startEnd, point1,robot)
    traj_planner = Traj_Planner;
    pause(0.1)
    traj01_a = traj_planner.cubic_traj(0,2,0,0,startEnd(1),point1(1));
    traj01_b = traj_planner.cubic_traj(0,2,0,0,startEnd(2),point1(2));
    traj01_c = traj_planner.cubic_traj(0,2,0,0,startEnd(3),point1(3));
    
    traj01 = [traj01_a; traj01_b; traj01_c];
    data = robot.run_trajectory(traj01,2,1)
end

function cubic_TS(robot)
    traj_planner = Traj_Planner;
    startEnd = [95; -95; 33]
    point1 = [176; -13; 158]
    point2 = [151 102 49]

    startEndDEG = robot.ik3001(startEnd)
%     startEnd = robot.ik3001(startEnd)
%     point1 = robot.ik3001(point1);
%     point2 = robot.ik3001(point2);
    
    robot.interpolate_jp(startEndDEG,2000)
    pause(2)
    traj01_a = traj_planner.cubic_traj(0,2,0,0,startEnd(1),point1(1));
    traj01_b = traj_planner.cubic_traj(0,2,0,0,startEnd(2),point1(2));
    traj01_c = traj_planner.cubic_traj(0,2,0,0,startEnd(3),point1(3));

    traj12_a = traj_planner.cubic_traj(0,2,0,0,point1(1),point2(1));
    traj12_b = traj_planner.cubic_traj(0,2,0,0,point1(2),point2(2));
    traj12_c = traj_planner.cubic_traj(0,2,0,0,point1(3),point2(3));

    traj20_a = traj_planner.cubic_traj(0,2,0,0,point2(1),startEnd(1));
    traj20_b = traj_planner.cubic_traj(0,2,0,0,point2(2),startEnd(2));
    traj20_c = traj_planner.cubic_traj(0,2,0,0,point2(3),startEnd(3));
    
    traj01 = [traj01_a; traj01_b; traj01_c]
    traj12 = [traj12_a; traj12_b; traj12_c]
    traj20 = [traj20_a; traj20_b; traj20_c]

    data1 = robot.run_trajectory(traj01,2,1);
    data2 = robot.run_trajectory(traj12,2,1);
    data2(:,1) = data2(:,1) + data1(end,1);
    data3 = robot.run_trajectory(traj20,2,1);
    data3(:,1) = data3(:,1) + data2(end,1);
    
    dlmwrite('jacobMov.csv',data1,'delimiter',',');
    dlmwrite('jacobMov.csv',data2,'delimiter',',','-append');
    dlmwrite('jacobMov.csv',data3,'delimiter',',','-append');
    
    disp(traj01)
    disp(traj12)
    disp(traj20)
end

function cubic_JS(robot)
    traj_planner = Traj_Planner;
    startEnd = [95; -95; 33]
    point1 = [176; -13; 158]
    point2 = [151 102 49]

    startEndDEG = robot.ik3001(startEnd)
    startEnd = robot.ik3001(startEnd)
    point1 = robot.ik3001(point1);
    point2 = robot.ik3001(point2);
    
    robot.interpolate_jp(startEndDEG,2000)
    pause(3)
    traj01_a = traj_planner.cubic_traj(0,2,0,0,startEnd(1),point1(1));
    traj01_b = traj_planner.cubic_traj(0,2,0,0,startEnd(2),point1(2));
    traj01_c = traj_planner.cubic_traj(0,2,0,0,startEnd(3),point1(3));

    traj12_a = traj_planner.cubic_traj(0,2,0,0,point1(1),point2(1));
    traj12_b = traj_planner.cubic_traj(0,2,0,0,point1(2),point2(2));
    traj12_c = traj_planner.cubic_traj(0,2,0,0,point1(3),point2(3));

    traj20_a = traj_planner.cubic_traj(0,2,0,0,point2(1),startEnd(1));
    traj20_b = traj_planner.cubic_traj(0,2,0,0,point2(2),startEnd(2));
    traj20_c = traj_planner.cubic_traj(0,2,0,0,point2(3),startEnd(3));
    
    traj01 = [traj01_a; traj01_b; traj01_c]
    traj12 = [traj12_a; traj12_b; traj12_c]
    traj20 = [traj20_a; traj20_b; traj20_c]

    data1 = robot.run_trajectory(traj01,2,0);
    data2 = robot.run_trajectory(traj12,2,0);
    data2(:,1) = data2(:,1) + data1(end,1);
    data3 = robot.run_trajectory(traj20,2,0);
    data3(:,1) = data3(:,1) + data2(end,1);
    
    dlmwrite('trajData5_pos_JointSpace_cubic.csv',data1,'delimiter',',');
    dlmwrite('trajData5_pos_JointSpace_cubic.csv',data2,'delimiter',',','-append');
    dlmwrite('trajData5_pos_JointSpace_cubic.csv',data3,'delimiter',',','-append');
    
    disp(traj01)
    disp(traj12)
    disp(traj20)
end

function quintic_TS(robot)
    traj_planner = Traj_Planner;
    startEnd = [95; -95; 33]
    point1 = [176; -13; 158]
    point2 = [151 102 49]

    startEndDEG = robot.ik3001(startEnd)
    
    moveTime = 2;
    robot.interpolate_jp(startEndDEG,2000)
    pause(3)
    
    traj01_a = traj_planner.quintic_traj(0,moveTime,0,0,startEnd(1),point1(1),0,0);
    traj01_b = traj_planner.quintic_traj(0,moveTime,0,0,startEnd(2),point1(2),0,0);
    traj01_c = traj_planner.quintic_traj(0,moveTime,0,0,startEnd(3),point1(3),0,0);

    traj12_a = traj_planner.quintic_traj(0,moveTime,0,0,point1(1),point2(1),0,0);
    traj12_b = traj_planner.quintic_traj(0,moveTime,0,0,point1(2),point2(2),0,0);
    traj12_c = traj_planner.quintic_traj(0,moveTime,0,0,point1(3),point2(3),0,0);

    traj20_a = traj_planner.quintic_traj(0,moveTime,0,0,point2(1),startEnd(1),0,0);
    traj20_b = traj_planner.quintic_traj(0,moveTime,0,0,point2(2),startEnd(2),0,0);
    traj20_c = traj_planner.quintic_traj(0,moveTime,0,0,point2(3),startEnd(3),0,0);

    traj01 = [traj01_a; traj01_b; traj01_c];
    traj12 = [traj12_a; traj12_b; traj12_c];
    traj20 = [traj20_a; traj20_b; traj20_c];
    
    curr_pos = robot.measured_js(1,0);
    robot.plot_armNODELAY(curr_pos(1,:));
    disp("paused")
    pause(5)
    
    
    data1 = robot.run_trajectoryPlot(traj01,moveTime,1);
    data2 = robot.run_trajectoryPlot(traj12,moveTime,1);
    data3 = robot.run_trajectoryPlot(traj20,moveTime,1);

    
    disp(data3)

    dlmwrite('jacobMov.csv',data1,'delimiter',',');
    data2(:,1) = data2(:,1)+2;
    dlmwrite('jacobMov.csv',data2,'delimiter',',','-append');
    data3(:,1) = data3(:,1)+4;
    dlmwrite('jacobMov.csv',data3,'delimiter',',','-append');
    
    disp(traj01)
    disp(traj12)
    disp(traj20)
end

function quintic_JS(robot)
    traj_planner = Traj_Planner;
    startEnd = [95; -95; 33]
    point1 = [176; -13; 158]
    point2 = [151 102 49]
    startEndDEG = robot.ik3001(startEnd)
    startEnd = robot.ik3001(startEnd)
    point1 = robot.ik3001(point1);
    point2 = robot.ik3001(point2);
    
    robot.interpolate_jp(startEndDEG,2000)
    pause(3)
    
    traj01_a = traj_planner.quintic_traj(0,2,0,0,startEnd(1),point1(1),0,0)
    traj01_b = traj_planner.quintic_traj(0,2,0,0,startEnd(2),point1(2),0,0)
    traj01_c = traj_planner.quintic_traj(0,2,0,0,startEnd(3),point1(3),0,0)

    traj12_a = traj_planner.quintic_traj(0,2,0,0,point1(1),point2(1),0,0);
    traj12_b = traj_planner.quintic_traj(0,2,0,0,point1(2),point2(2),0,0);
    traj12_c = traj_planner.quintic_traj(0,2,0,0,point1(3),point2(3),0,0);

    traj20_a = traj_planner.quintic_traj(0,2,0,0,point2(1),startEnd(1),0,0);
    traj20_b = traj_planner.quintic_traj(0,2,0,0,point2(2),startEnd(2),0,0);
    traj20_c = traj_planner.quintic_traj(0,2,0,0,point2(3),startEnd(3),0,0);

    traj01 = [traj01_a; traj01_b; traj01_c];
    traj12 = [traj12_a; traj12_b; traj12_c];
    traj20 = [traj20_a; traj20_b; traj20_c];
    
    data1 = robot.run_trajectory(traj01,2,0);
    data2 = robot.run_trajectory(traj12,2,0);
    data2(:,1) = data2(:,1) + data1(end,1);
    data3 = robot.run_trajectory(traj20,2,0);
    data3(:,1) = data3(:,1) + data2(end,1);
    
    disp(data3)

    dlmwrite('trajData7_pos_JointSpace_quintic.csv',data1,'delimiter',',');
    dlmwrite('trajData7_pos_JointSpace_quintic.csv',data2,'delimiter',',','-append');
    dlmwrite('trajData7_pos_JointSpace_quintic.csv',data3,'delimiter',',','-append');
    
    disp(traj01)
    disp(traj12)
    disp(traj20)
end

function testOne(robot,point)
    disp("Forward Kinematics for inputted point")
    angles = robot.ik3001(transpose(point));
    disp("Inverse Kinematics Angles")
    disp(angles)
    result = robot.fk3001(angles);
    disp("Forward Kinematics : ")
    disp(result)
    
end


%takes in MM and returns ang
function ikTriangles(robot)
    startEnd = [95; -95; 33]
    point1 = [176; -13; 158]
    point2 = [151 102 49]
    point1 = transpose(robot.ik3001(point1));
    point2 = transpose(robot.ik3001(point2));
    startEnd = robot.ik3001(startEnd);
    robot.interpolate_jp(transpose(startEnd),2500);
    pause(3)

    data1 = waitUninterpolatedReturnDataPlotMM(point1,robot);
    
    data2 = waitUninterpolatedReturnDataPlotMM(point2, robot);
    data2(:,4) = data2(:,4) + data1(end,4);

    data3 = waitUninterpolatedReturnDataPlotMM(startEnd, robot);
    data3(:,4) = data3(:,4) + data2(end,4);
    
    dlmwrite('triangle.csv',data1,'delimiter',',');
    dlmwrite('triangle.csv',data2,'delimiter',',','-append');
    dlmwrite('triangle.csv',data3,'delimiter',',','-append');
end

function ikTrianglesANG(robot)
%     startEnd = [39.5072; -119.8798; 46.4594]
%     point1 = [171.8705; -2.8800; 191.7194]
%     point2 = [31.8431; 128.4797; 36.3015]

    startEnd = [95; -95; 33]
    point1 = [176; -13; 158]
    point2 = [151 102 49]


    point1 = transpose(robot.ik3001(point1));
    point2 = transpose(robot.ik3001(point2));
    startEnd = transpose(robot.ik3001(startEnd));
    robot.interpolate_jp(startEnd,2500);
    pause(3)
    
    data1 = waitUninterpolatedReturnDataPlot(point1,robot);
    
    data2 = waitUninterpolatedReturnDataPlot(point2, robot);
    data2(:,4) = data2(:,4) + data1(end,4);

    data3 = waitUninterpolatedReturnDataPlot(startEnd, robot);
    data3(:,4) = data3(:,4) + data2(end,4);
    
    dlmwrite('triangleANG.csv',data1,'delimiter',',');
    dlmwrite('triangleANG.csv',data2,'delimiter',',','-append');
    dlmwrite('triangleANG.csv',data3,'delimiter',',','-append');
end

%--------------------------------------
function waitUnInterpolated(goal,robot)
    robot.servo_jp(goal);
    curr_pos = robot.measured_js(1, 0)
    target_diff = abs(curr_pos(1,:) - robot.goal_js())
    while(any(target_diff>1))
    curr_pos = robot.measured_js(1, 0);
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    end
end

function waitInterpolated(goal,time,robot)
    robot.interpolate_jp(goal,time);
    curr_pos = robot.measured_js(1, 0)
    target_diff = abs(curr_pos(1,:) - robot.goal_js())
    while(any(target_diff>1))
    curr_pos = robot.measured_js(1, 0);
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    end
end

function data = waitInterpolatedReturnData(goal,time,robot)
robot.interpolate_jp(goal, time);
    tStart = tic;
    curr_pos = robot.measured_js(1, 0);
    data = [curr_pos(1,:) toc];
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    while(any(target_diff>1))
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        curr_pos = robot.measured_js(1, 0);
        data = [data; curr_pos(1,:) toc];
    end
end

function waitUnInterpolatedPlot(goal,robot)
    robot.servo_jp(goal);
    curr_pos = robot.measured_js(1, 0)
    target_diff = abs(curr_pos(1,:) - robot.goal_js())
    while(any(target_diff>1))
        curr_pos = robot.measured_js(1, 0);
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        robot.plot_arm(curr_pos(1,:).')
    end
end

function waitInterpolatedPlot(goal,time,robot)
    robot.interpolate_jp(goal,time);
    curr_pos = robot.measured_js(1, 0)
    target_diff = abs(curr_pos(1,:) - robot.goal_js())
    while(any(target_diff>2))
        curr_pos = robot.measured_js(1, 0);
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        robot.plot_arm(curr_pos(1,:).')
    end
    disp("Point 0")
    disp(robot.fk3001(transpose(goal)))
    disp(robot.setpoint_js())
   
end

function data = waitInterpolatedReturnDataPlot(goal,time,robot)
robot.interpolate_jp(goal, time);
    tic;
    curr_pos = robot.measured_js(1, 0);
    armPos = robot.arm_data(curr_pos(1,:).');
    data = [curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)];
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    while(any(target_diff>2))
        curr_pos = robot.measured_js(1, 0);
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        armPos = robot.arm_data(curr_pos(1,:).');
        %disp([curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)]);
        data = [data; [curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)]];

        disp(curr_pos(1,:))
    end
end

function data = waitInterpolatedReturnDataPlotMM(goal,time,robot)
robot.interpolate_jp(goal, time);
    tic;
    curr_pos = robot.measured_js(1, 0);
    armPos = robot.arm_data(curr_pos(1,:).');
    dataMM = robot.ik3001(transpose(armPos))
    data = [transpose(dataMM) toc];
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    while(any(target_diff>2))
        curr_pos = robot.measured_js(1, 0);
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        armPos = robot.arm_data(curr_pos(1,:).');
        %disp([curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)]);
        data = [data; [curr_pos(1,:) toc]];
        disp(curr_pos(1,:))
    end
end

function data = waitUninterpolatedReturnDataPlot(goal,robot)
robot.servo_jp(goal);
    tic;
    curr_pos = robot.measured_js(1, 0);
    armPos = robot.arm_data(curr_pos(1,:).');
    data = [curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)];
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    while(any(target_diff>2))
        curr_pos = robot.measured_js(1, 0);
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        armPos = robot.arm_data(curr_pos(1,:).');
        %disp([curr_pos(1,:) toc armPos(1,:) armPos(2,:) npwarmPos(3,:) armPos(4,:) armPos(5,:)]);
        data = [data; [curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)]];
        disp(curr_pos(1,:))
    end
end

function data = waitUninterpolatedReturnDataPlotMM(goal,robot)
robot.servo_jp(goal);
    tic;
    curr_posDeg = robot.measured_js(1, 0);
    pos = transpose(curr_posDeg(1,:));
    mm = robot.fk3001(pos);
    dataMM = mm(:,4);
    target_diff = abs(curr_posDeg(1,:) - robot.goal_js())
    data = [transpose(dataMM) toc];
    
    while(any(target_diff>2))
    curr_posDeg = robot.measured_js(1, 0);
    target_diff = abs(curr_posDeg(1,:) - robot.goal_js());
    pos = transpose(curr_posDeg(1,:));
    mm = robot.fk3001(pos);
    dataMM = mm(:,4);
    data = [data; transpose(dataMM) toc]
    end
end

function checkPosConstantly(time,robot)
    %start = tic;
    while(true) %tok < start + time)
        robot.measured_js(1,0)
        pause(0.1)
        
    end
end

function checkVelConstantly(robot)
    %start = tic;
    while(true) %tok < start + time)
        robot.measured_js(1,1) 
        pause(0.2)
    end
end

function checkJacobConstantly(robot)
    while(true) %tok < start + time)
        pos = robot.measured_js(1,0);
        pos = pos(1,:);
        cp = robot.measured_js(1,1);
        e = fdk3001(robot,cp);
        pause(0.1)
    end
end


function checkPosConstantlyMM(robot)
    while(true)
        pos = robot.measured_js(1,0);
        pos = transpose(pos(1,:));
        mm = robot.fk3001(pos);
        disp(pos)
        disp(mm)
        pause(0.3)
    end
end
