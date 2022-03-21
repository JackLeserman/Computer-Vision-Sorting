% RBE3001 - Laboratory 2

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
    disp "Started"
    L0 = 55;
    L1 = 40;
    L2 = 100;
    L3 = 100;
    
%     robot.servo_jp([0 0 0])
%     pause(2)
% %     
%    try
%        view([45 45 45])
%        while true
%           pos = robot.measured_js(1,0)
%           disp(pos(1,:).')
%           %robot.plot_arm(pos(1,:).');
%        end
%   catch exception
%     getReport(exception)
%     disp('Exited on error, clean shutdown');
%    end
  
target1 =[60 -40 20]
target2 =[-40 63 -30]
target3 =[-56 -31 10]
target4 =[96 17 -60]
target5 =[-42 74 -43]


pEightI = [0 55.6600 18.3500]
pEightII = [0 33.5800 -63.4900]
pEightIII = [0 40.5800 39.7100]

pEightIa = [0 60 30]
pEightIIa = [0 50 -70]
pEightIIIa = [0 45 50]

fk = robot.fk3001([80 0 85])
disp("FK @ 80 0 85")
disp(fk)
% %%partFive-----------------
% t
% robot.plot_arm(target(:,1));

%robot.workspace()
    
% Part 7

%waitInterpolatedPlot(target1,3000,robot)
%waitInterpolatedPlot(target2,3000,robot)
%waitInterpolatedPlot(target3,3000,robot)
%waitInterpolatedPlot(target4,3000,robot)
%waitInterpolatedPlot(target5,3000,robot)
%waitInterpolatedPlot([0 0 0],1000,robot)

    
% %part4----------------------------------------------
%     target = [24 20 62];
%     home = [0 0 0];
%     
%     robot.interpolate_jp(home,2000);
%     pause(4)
%     
%     bigData = zeros(4,4,10);
%    
%     
%     for i = 1:10
%         robot.interpolate_jp(target,2000);
%         pause(3);
%         robot.interpolate_jp(home,2000);
%         pause(3);
%         data = robot.measured_cp()
%         bigData(:,:,i) = data;
%     end
% 
%     plotBoi = zeros(4,10)
%     plotBoi(4,:) = 1
%    
%     for i = 1:10
%         plotBoi2 = plotBoi(:,i)
%         bigData2 = bigData(:,:,i)
%         plotBoi(:,i) = bigData2 * plotBoi2
%     end
%     csvwrite('homePosTransform(Part4).csv',plotBoi);
%    disp(plotBoi)
    
    %end part4----------------------------------
    
    


%Clear up memory upon termination
robot.shutdown()

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
    armPos = robot.arm_data(curr_pos(1,:).')
    data = [curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)];
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    while(any(target_diff>2))
        curr_pos = robot.measured_js(1, 0);
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        armPos = robot.arm_data(curr_pos(1,:).')
        disp([curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)])
        data = [data; [curr_pos(1,:) toc armPos(1,:) armPos(2,:) armPos(3,:) armPos(4,:) armPos(5,:)]];
    end
end


function checkPosConstantly(time,robot)
    start = tic;
    while(toc < start + time)
        robot.measured_js(1,0)
    end
end


function partEight(point1,point2,robot)
    home = [0 0 0]
    data1 = waitInterpolatedReturnDataPlot(point1, 2000, robot)
    data2 = waitInterpolatedReturnDataPlot(point2, 2000, robot)
    data2(:,4) = data2(:,4) +2;
    data3 = waitInterpolatedReturnDataPlot(home, 2000, robot)
    data3(:,4) = data3(:,4) +4;
    dlmwrite('part8_3a.csv',data1,'delimiter',',');
    dlmwrite('part8_3b.csv',data2,'delimiter',',','-append');
    dlmwrite('part8_3c.csv',data3,'delimiter',',','-append');
    
end
