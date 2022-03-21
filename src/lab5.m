%{
Notes Go Here
=======================
topLeft = -98 96 -70
bottomLeft = -77 96 -70
topRight = 98 96 -70
bottomRight = 77 96 -70

%}
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');
format short
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
cam = Camera();
intrins = cam.cam_IS;
%------------------Run Tests Here-------------------

try
    image = cam.getImage();
    disp("Place balls. Hit any key to continue, but press H, we love the H key");
    disp("Also, dont forget to calibrate the arm, you fuckwit");
    disp("Also, have a nice day :D");
    
    [y, Fs] = audioread('startup.mp3');
    player = audioplayer(y, Fs);
    play(player);
    pause;
    stillRunning = true;
    while(stillRunning)
        if(collectBall(intrins,"red", cam, robot))
            disp("Collecting Red Ball")
        elseif(collectBall(intrins,"orange", cam, robot))
            disp("Collecting Orange Ball")
        elseif(collectBall(intrins,"yellow", cam, robot))
            disp("Collecting Yellow Ball")
        elseif(collectBall(intrins,"green", cam, robot))
            disp("Collecting Green Ball")
        else
            disp("No Balls Found!")
            prompt = 'If you would like to continue, press 1. To end, press 2.';
            x = input(prompt)
            if(x == 1)
                disp("You chose to continue")
                disp("Press any button to continue once you place your ball / balls on the board") % Fuck you lance armstrong
                pause;
            else
                disp("Where where you when robot was kil?")
                stillRunning = false;
            end
        end
    end
    [y, Fs] = audioread('yay.mp3');
    player = audioplayer(y, Fs);
    play(player);

catch exception
    [y, Fs] = audioread('error.mp3');
    player = audioplayer(y, Fs);
    play(player);
    getReport(exception)
    disp('Command error');
end

%------------- -----Test Functions-----------------------  

%{
Extrinsics
----------
Rotation vectors:
                         [   -1.0622 +/- 0.0009        0.1179 +/- 0.0009        0.1892 +/- 0.0006  ]
                         [   -1.0736 +/- 0.0009        0.1141 +/- 0.0009        0.1939 +/- 0.0006  ]
                         [   -1.0893 +/- 0.0009        0.1141 +/- 0.0009        0.2048 +/- 0.0006  ]
                         [   -1.0812 +/- 0.0009        0.1501 +/- 0.0009        0.2790 +/- 0.0006  ]
                         [   -1.0023 +/- 0.0010        0.1131 +/- 0.0010        0.1547 +/- 0.0005  ]
                         [   -1.0317 +/- 0.0010        0.0412 +/- 0.0009        0.1286 +/- 0.0005  ]
                         [   -1.0024 +/- 0.0010        0.0543 +/- 0.0010        0.1406 +/- 0.0005  ]
                         [   -1.0467 +/- 0.0010        0.0954 +/- 0.0010        0.1126 +/- 0.0005  ]
                         [   -1.0694 +/- 0.0009        0.0652 +/- 0.0009        0.1176 +/- 0.0005  ]
                         [   -1.0474 +/- 0.0010        0.0036 +/- 0.0009        0.0490 +/- 0.0005  ]
                         [   -1.0174 +/- 0.0010        0.0125 +/- 0.0010        0.0375 +/- 0.0005  ]
                         [   -1.0174 +/- 0.0010       -0.3111 +/- 0.0009       -0.2081 +/- 0.0006  ]
                         [   -0.9966 +/- 0.0010       -0.3544 +/- 0.0009       -0.2443 +/- 0.0005  ]
                         [   -0.9788 +/- 0.0010       -0.3802 +/- 0.0009       -0.3049 +/- 0.0005  ]
                         [   -0.9568 +/- 0.0010       -0.4098 +/- 0.0009       -0.3235 +/- 0.0005  ]
                         [   -0.9176 +/- 0.0010       -0.4473 +/- 0.0009       -0.3796 +/- 0.0005  ]
                         [   -0.9108 +/- 0.0010       -0.4433 +/- 0.0009       -0.3624 +/- 0.0005  ]
                         [   -0.8925 +/- 0.0011       -0.0996 +/- 0.0010       -0.2297 +/- 0.0006  ]
                         [   -0.9064 +/- 0.0011       -0.0341 +/- 0.0010       -0.1972 +/- 0.0006  ]
                         [   -1.0061 +/- 0.0010       -0.0899 +/- 0.0009       -0.1479 +/- 0.0006  ]
                         [   -1.0064 +/- 0.0010       -0.0724 +/- 0.0009       -0.1388 +/- 0.0006  ]
                         [   -0.9965 +/- 0.0010       -0.0944 +/- 0.0009       -0.1275 +/- 0.0006  ]
                         [   -1.0298 +/- 0.0010        0.0283 +/- 0.0009       -0.0625 +/- 0.0006  ]
                         [   -1.0357 +/- 0.0010        0.0876 +/- 0.0009       -0.0232 +/- 0.0006  ]

Translation vectors (millimeters):
                         [ -106.3226 +/- 0.2398      -66.9002 +/- 0.2388      361.9861 +/- 0.2814  ]
                         [ -112.0207 +/- 0.2493      -64.7053 +/- 0.2499      378.8932 +/- 0.2985  ]
                         [ -130.7313 +/- 0.2539      -59.2575 +/- 0.2593      388.9658 +/- 0.3173  ]
                         [ -111.3475 +/- 0.2621      -65.4585 +/- 0.2625      399.1572 +/- 0.3109  ]
                         [  -78.0065 +/- 0.2474      -34.8065 +/- 0.2399      376.3317 +/- 0.2842  ]
                         [ -100.4707 +/- 0.2427      -37.2362 +/- 0.2406      372.1007 +/- 0.2948  ]
                         [  -89.6031 +/- 0.2429      -43.0530 +/- 0.2385      370.6832 +/- 0.2891  ]
                         [  -83.2463 +/- 0.2455      -22.6711 +/- 0.2386      374.9047 +/- 0.2867  ]
                         [  -98.2171 +/- 0.2435      -20.9068 +/- 0.2402      373.5601 +/- 0.2919  ]
                         [ -126.5134 +/- 0.2348      -26.4732 +/- 0.2401      363.6967 +/- 0.3066  ]
                         [  -92.9886 +/- 0.2385      -35.5485 +/- 0.2348      365.4303 +/- 0.2950  ]
                         [ -152.2471 +/- 0.2128      -31.5164 +/- 0.2281      333.9191 +/- 0.3484  ]
                         [ -141.0837 +/- 0.2052      -31.8237 +/- 0.2175      321.2223 +/- 0.3368  ]
                         [ -133.5034 +/- 0.2018      -32.9793 +/- 0.2121      315.3670 +/- 0.3345  ]
                         [ -123.2450 +/- 0.2017      -32.1020 +/- 0.2094      315.0859 +/- 0.3345  ]
                         [ -127.3952 +/- 0.1968      -36.4065 +/- 0.2057      306.5352 +/- 0.3377  ]
                         [ -131.6211 +/- 0.2014      -38.6038 +/- 0.2110      313.7472 +/- 0.3469  ]
                         [ -104.2615 +/- 0.2423      -60.3029 +/- 0.2439      372.7166 +/- 0.3458  ]
                         [ -113.1503 +/- 0.2425      -57.8250 +/- 0.2445      371.8676 +/- 0.3344  ]
                         [ -129.8291 +/- 0.2253      -53.9201 +/- 0.2328      347.9679 +/- 0.3195  ]
                         [ -137.3465 +/- 0.2262      -43.2475 +/- 0.2357      351.2739 +/- 0.3229  ]
                         [ -151.4144 +/- 0.2246      -44.7893 +/- 0.2390      350.3887 +/- 0.3340  ]
                         [ -127.3690 +/- 0.2399      -30.7583 +/- 0.2449      371.2272 +/- 0.3180  ]
                         [ -127.1908 +/- 0.2511      -34.4211 +/- 0.2552      387.3630 +/- 0.3238  ]
%}


function exists = collectBall(intrins, color, cam, robot)
    try
        tchecker = [0 1 0 50; 1 0 0 -100; 0 0 -1 0; 0 0 0 1];  
        traj_planner = Traj_Planner;
        rm1 = [0.996047724937052  -0.032406167376463   0.082696855827127];
        rm2 = [ 0.088704386159434   0.410357632558937  -0.907600212250252];
        rm3 = [ -0.004523441588193   0.911348700397709   0.411610355505793];
        rotation_matrix = [rm1;rm2;rm3];
        translation_vector = [(1.0e+02 * -1.318741161884955) (1.0e+02 * -0.420437753432049) (1.0e+02 * 3.456607537704431)];
        
        image2 = cam.getImage();
        figure
        imshow(image2)
        [position, radius] = cam.imageToFilteredHSV(image2, color)
        if(radius == 0)
            exists = false;
            disp(color + " Not found")
        else
            disp(position)
            ptwValue = pointsToWorld(intrins,rotation_matrix,translation_vector, position);
            ballCheckerSpot = checkerAdjust(ptwValue);
            worldPoint = tchecker * [ballCheckerSpot(1); ballCheckerSpot(2); 0; 1];
            x = worldPoint(1);
            y = worldPoint(2);
            z = worldPoint(3) + 30;
            z2 = worldPoint(3) + 10;
            worldPointBefore = [x y z];
            worldPointAfter = [x y z2];
            worldPointBefore = robot.ik3001(worldPointBefore);
            worldPointAfter = robot.ik3001(worldPointAfter);

            quintic_JS([0 0 0],3,robot);
            robot.openGripper();
            pause(1)
            quintic_JS(worldPointBefore,3,robot);
            quintic_JS(worldPointAfter,1,robot);
            robot.closeGripper();
            pause(1);
            startEnd = robot.measured_js(1,0);
            curr = startEnd(1,:);
            robot.fk3001(transpose(curr))
            quintic_JS(worldPointBefore,1,robot);
            quintic_JS([0 0 0],3,robot);
            pause(1);
            robot.deliverOrb(color)
            pause(1);
            quintic_JS([0 0 0],3,robot);
            pause(1);
            
            exists = true;
        end

    catch exception
        [y, Fs] = audioread('error.mp3');
        player = audioplayer(y, Fs);
        play(player);
        getReport(exception)
        disp('Command error');
    end
end

function exists = collectPat(intrins, cam, robot)
    try
        tchecker = [0 1 0 50; 1 0 0 -100; 0 0 -1 0; 0 0 0 1];  
        traj_planner = Traj_Planner;
        rm1 = [0.996047724937052  -0.032406167376463   0.082696855827127];
        rm2 = [ 0.088704386159434   0.410357632558937  -0.907600212250252];
        rm3 = [ -0.004523441588193   0.911348700397709   0.411610355505793];
        rotation_matrix = [rm1;rm2;rm3];
        translation_vector = [(1.0e+02 * -1.318741161884955) (1.0e+02 * -0.420437753432049) (1.0e+02 * 3.456607537704431)];
        
        image2 = cam.getImage();
        figure
        imshow(image2)
        [position, radius] = cam.imageToFilteredHSV(image2, color)
        if(radius == 0)
            exists = false;
            disp(color + " Not found")
        else
            disp(position)
            ptwValue = pointsToWorld(intrins,rotation_matrix,translation_vector, position);
            ballCheckerSpot = checkerAdjust(ptwValue);
            worldPoint = tchecker * [ballCheckerSpot(1); ballCheckerSpot(2); 0; 1];
            x = worldPoint(1);
            y = worldPoint(2);
            z = worldPoint(3) + 30;
            z2 = worldPoint(3) + 10;
            worldPointBefore = [x y z];
            worldPointAfter = [x y z2];
            worldPointBefore = robot.ik3001(worldPointBefore);
            worldPointAfter = robot.ik3001(worldPointAfter);

            quintic_JS([0 0 0],3,robot);
            robot.openGripper();
            pause(1)
            quintic_JS(worldPointBefore,3,robot);
            quintic_JS(worldPointAfter,1,robot);
            robot.closeGripper();
            pause(1);
            startEnd = robot.measured_js(1,0);
            curr = startEnd(1,:);
            robot.fk3001(transpose(curr))
            quintic_JS(worldPointBefore,1,robot);
            quintic_JS([0 0 0],3,robot);
            pause(1);
            robot.deliverOrb(color)
            pause(1);
            quintic_JS([0 0 0],3,robot);
            pause(1);
            
            exists = true;
        end

    catch exception
        [y, Fs] = audioread('error.mp3');
        player = audioplayer(y, Fs);
        play(player);
        getReport(exception)
        disp('Command error');
    end
end

function checkerVal = checkerAdjust(ptwValue, robot)
    cameraPointOnChecker = [118 413.5];
    vectorFromBallToCamera = [(cameraPointOnChecker(1) - ptwValue(1)) (cameraPointOnChecker(2) - ptwValue(2))];
    adjustmentVector = vectorFromBallToCamera * 0.06;
    checkerVal = ptwValue + adjustmentVector;
end

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

% function quintic_TS(point1,robot)
%     traj_planner = Traj_Planner;
%     point1 = point
%     startEnd = robot.measured_js(1,0);
%     startEnd = startEnd(1,:);
%     point1 = robot.ik3001(point1);
%     startEnd = robot.ik3001(startEnd)
%     moveTime = 2.5;
%     traj01_a = traj_planner.quintic_traj(0,moveTime,0,0,startEnd(1),point1(1),0,0);
%     traj01_b = traj_planner.quintic_traj(0,moveTime,0,0,startEnd(2),point1(2),0,0);
%     traj01_c = traj_planner.quintic_traj(0,moveTime,0,0,startEnd(3),point1(3),0,0);
% 
%     traj01 = [traj01_a; traj01_b; traj01_c];
%     
%     robot.run_trajectory(traj01,moveTime,1);
% end

function quintic_JS(point,time,robot)
    traj_planner = Traj_Planner;
    startEnd = robot.measured_js(1,0);
    startEnd = startEnd(1,:)
    point1 = point;
    
    traj01_a = traj_planner.quintic_traj(0,time,0,0,startEnd(1),point1(1),0,0)
    traj01_b = traj_planner.quintic_traj(0,time,0,0,startEnd(2),point1(2),0,0)
    traj01_c = traj_planner.quintic_traj(0,time,0,0,startEnd(3),point1(3),0,0)

    traj01 = [traj01_a; traj01_b; traj01_c];
    
    robot.run_trajectory(traj01,time,0);
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

function checkPosConstantly(robot)
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
