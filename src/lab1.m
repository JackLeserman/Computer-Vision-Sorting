%%
% RBE3001 - Laboratory 1 
% beep boop andy was here
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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
    format long g
    %Set to 0
    waitInterpolated([0 0 0], 1000,robot);
    
    %Part 4:
    %Go to [45 0 0] 3 times interpolated, then 3 times not interpolated and
    %save it all to a file
    save_data = waitInterpolated([45 0 0],3000,robot);
    csvwrite('jointimeInterpolated1.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitInterpolated([45 0 0],3000,robot);
    csvwrite('jointimeInterpolated2.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitInterpolated([45 0 0],3000,robot);
    csvwrite('jointimeInterpolated3.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitUnInterpolated([45 0 0],3000,robot);
    csvwrite('jointimeNOTInterpolated1.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitUnInterpolated([45 0 0],3000,robot);
    csvwrite('jointimeNOTInterpolated2.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitUnInterpolated([45 0 0],3000,robot);
    csvwrite('jointimeNOTInterpolated3.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot); 
    
    %Part 5:
    
    target1 = [-32.49 15.16 -10.40];
    target2 = [17.28 42.78  12.45];
    target3 = [-14.51 -22.12 13.49];
    target4 = [42.0 5.69 16.61];
    
    %starts at 0 0 0, go to each of its places both interpolated and not
    %Target 1
    save_data = waitInterpolatedReturnData(target1,3000,robot);
    csvwrite('position1Interpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitUnInterpolatedReturnData(target1);
    csvwrite('position1UnInterpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    %Target 2
    save_data = waitInterpolatedReturnData(target2,3000,robot);
    csvwrite('position2Interpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitUnInterpolatedReturnData(target2);
    csvwrite('position2UnInterpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    %Target 3
    save_data = waitInterpolatedReturnData(target3,3000,robot);
    csvwrite('position3Interpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitUnInterpolatedReturnData(target3);
    csvwrite('position3UnInterpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    %Target 4
    save_data = waitInterpolatedReturnData(target4,3000,robot);
    csvwrite('position4Interpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    save_data = waitUnInterpolatedReturnData(target4);
    csvwrite('position4UnInterpolated.csv',save_data);
    waitInterpolated([0 0 0], 1000,robot);
    
    
%     
%     robot.interpolate_jp(target1, 3000);
%     tStart = tic;
%     curr_pos = robot.measured_js(1, 0);
%     csv_data = [curr_pos(1,:) toc];
%     bruh2 = abs(curr_pos(1,:) - robot.goal_js());
%     while(any(bruh2>2))
%         bruh2 = abs(curr_pos(1,:) - robot.goal_js());
%         curr_pos = robot.measured_js(1, 0);
%         csv_data = [csv_data; curr_pos(1,:) toc];
%     end
%     csvwrite('jointimeInterpolated1.csv',csv_data)
%     plot(csv_data(:,1),csv_data(:,4),"b*")
%     hold on
%     plot(csv_data(:,2),csv_data(:,4),"ro")
%     plot(csv_data(:,3),csv_data(:,4),"gs")
%     legend("Motor1 Pos", "Motor2 Pos", "Motor3 Pos")
%     hold off
%     xlabel('Time in ms')
%     
%     
%     %Reset and go non-interpolated
%     robot.servo_jp([0 0 0]);
%     curr_pos = robot.measured_js(1, 0)
%     bruh = abs(curr_pos(1,:) - robot.goal_js())
%     while(any(bruh>2))
%     curr_pos = robot.measured_js(1, 0);
%     bruh = abs(curr_pos(1,:) - robot.goal_js());
%     end
%     
%     robot.servo_jp(target1);
%     tStart = tic;
%     curr_pos = robot.measured_js(1, 0);
%     csv_data = [curr_pos(1,:) toc];
%     bruh2 = abs(curr_pos(1,:) - robot.goal_js());
%     while(any(bruh2>2))
%         bruh2 = abs(curr_pos(1,:) - robot.goal_js());
%         curr_pos = robot.measured_js(1, 0);
%         csv_data = [csv_data; curr_pos(1,:) toc];
%     end
%     csvwrite('jointimeUninterpolated1.csv',csv_data)
%     plot(csv_data(:,1),csv_data(:,4),"b*")
%     hold on
%     plot(csv_data(:,2),csv_data(:,4),"ro")
%     plot(csv_data(:,3),csv_data(:,4),"gs")
%     legend("Motor1 Pos", "Motor2 Pos", "Motor3 Pos")
%     hold off
%     xlabel('Time in ms')
%     

% try
%   disp "started"
%   %Part 4 1 through 3, 5 has to be done separately
%     
%     %Go to position 0
%     robot.servo_jp([0 0 0]);
%     disp("e")
%     curr_pos = robot.measured_js(1, 0);
%     bruh = abs(curr_pos(1,:) - robot.goal_js());
%     %disp(bruh)
%     disp(curr_pos)
%     while(any(bruh>2))
%     
%     curr_pos = robot.measured_js(1, 0);
%     bruh = abs(curr_pos(1,:) - robot.goal_js());
%     disp("AnyBruh")
%     disp(any(bruh>2))
%     disp("---")
%     disp("Bruh")
%     disp(bruh > 2)
%     disp("---")
%     disp("GOAL")
%     disp(robot.goal_js())
%     disp("---")
%     disp("pos")
%     disp(curr_pos(1,:))
%    end
%     %Send robot's base angle from 0degrees to 45 with interpolation of 3
%     seconds
%     robot.servo_jp([45 0 0]);
%     
%     tStart = tic;
%     curr_pos = robot.measured_js(1, 0);
%     csv_data = [curr_pos(1,:) toc];
%     bruh2 = abs(curr_pos(1,:) - robot.goal_js());
%     while(any(bruh2>2))
%         bruh2 = abs(curr_pos(1,:) - robot.goal_js());
%         curr_pos = robot.measured_js(1, 0);
%         csv_data = [csv_data; curr_pos(1,:) toc];
%     end
%     csvwrite('jointime3-NOINT.csv',csv_data)
%     plot(csv_data(:,1),csv_data(:,4),"b*")
%     hold on
%     plot(csv_data(:,2),csv_data(:,4),"ro")
%     plot(csv_data(:,3),csv_data(:,4),"gs")
%     legend("Motor1 Pos", "Motor2 Pos", "Motor3 Pos")
%     hold off
%     xlabel('Time in ms')
%     
%     plot(csv_data1(:,1),csv_data1(:,4),"b*")
%     hold on
%     plot(csv_data2(:,2),csv_data2(:,4),"ro")
%     plot(csv_data3(:,3),csv_data3(:,4),"gs")

%   for k = viaPts
%       tic
%       packet = zeros(15, 1, 'single');
%       packet(1) = 1000;%one second time
%       packet(2) = 0;%linear interpolation
%       packet(3) = k;
%       packet(4) = 0;% Second link to 0
%       packet(5) = 0;% Third link to 0
% 
%       % Send packet to the server and get the response      
%       %robot.write sends a 15 float packet to the micro controller
%        robot.write(SERV_ID, packet); 
%        %robot.read reads a returned 15 float backet from the micro controller.
%        returnPacket = robot.read(SERVER_ID_READ);
%       toc
% 
%       if DEBUG
%           disp('Sent Packet:')
%           disp(packet);
%           disp('Received Packet:');
%           disp(returnPacket);
%       end
%       
%       toc
%       pause(1) 
%       
%   end
%   
%   % Closes then opens the gripper
%   robot.closeGripper()
%   pause(1)
%   robot.openGripper()
%   
% catch exception
%     getReport(exception)
%     disp('Exited on error, clean shutdown');
% end
%arr = [0 0 0];

%--------------TEST CASES----------------

%robot.servo_jp(arr);
%robot.interpolate_jp(arr,2000)
%robot.measured_js(true,true)
%robot.setpoint_js()
%robot.goal_js();


%Clear up memory upon termination
robot.shutdown()

function waitUnInterpolated(goal,robot)
    robot.servo_jp(goal);
    curr_pos = robot.measured_js(1, 0)
    target_diff = abs(curr_pos(1,:) - robot.goal_js())
    while(any(target_diff>2))
    curr_pos = robot.measured_js(1, 0);
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    end
end

function waitInterpolated(goal,time,robot)
    robot.interpolate_jp(goal,time);
    curr_pos = robot.measured_js(1, 0)
    target_diff = abs(curr_pos(1,:) - robot.goal_js())
    while(any(target_diff>2))
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
    while(any(target_diff>2))
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        curr_pos = robot.measured_js(1, 0);
        data = [data; curr_pos(1,:) toc];
    end
end

function data = waitUnInterpolatedReturnData(goal,robot)
robot.servo_jp(goal);
    tStart = tic;
    curr_pos = robot.measured_js(1, 0);
    data = [curr_pos(1,:) toc];
    target_diff = abs(curr_pos(1,:) - robot.goal_js());
    while(any(target_diff>2))
        target_diff = abs(curr_pos(1,:) - robot.goal_js());
        curr_pos = robot.measured_js(1, 0);
        data = [data; curr_pos(1,:) toc];
    end
end
