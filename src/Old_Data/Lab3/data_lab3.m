clear
data = csvread('BONUS_mov6.csv')
part6_plot(data)
% Part 5
% figure('NumberTitle','off','Name',"Part 5 Cubic Joint Space Trajectory Subplots")
% tiledlayout(1,3);
% dataaaa = csvread('trajData5_pos_JointSpace_cubic.csv');
% part5to7(dataaaa)
% title('Acceleration vs Time Data Cubic Joint Space Trajectory')

% Part 6
% figure('NumberTitle','off','Name',"Part 6 Cubic Task Space Trajectory Subplots")
% tiledlayout(1,3);
% dataaaa = csvread('trajData6_pos_TaskSpace_cubic.csv');
% part5to7(dataaaa)
% title('Acceleration vs Time Data Cubic Task Space Trajectory')

% figure('NumberTitle','off','Name',"Part 6 3D Plot With Unique Lines")
% tiledlayout(1,3);
% nexttile
% part6_plot(dataaaa)
% title('Path Followed Cubic Interpolation and Task Space')
% 
% nexttile
% dataaaa = csvread('trajData5_pos_JointSpace_cubic.csv');
% part6_plot(dataaaa)
% title('Path Followed Cubic Interpolation and Joint Space')
% 
% nexttile
% dataaaa = csvread('trajData5_pos_JointSpace_cubic.csv');
% part6_plot(dataaaa)
% title('Path Followed No Onboard Interpolation')

% Part 7
% figure('NumberTitle','off','Name',"Part 7 Quintic Joint Space Trajectory Subplots")
% tiledlayout(1,3);
% dataaaa = csvread('trajData7_pos_JointSpace_quintic.csv');
% part5to7(dataaaa)
% title('Acceleration vs Time Data Quintic Joint Space Trajectory')

% figure('NumberTitle','off','Name',"Part 7 Quintic Task Space Trajectory Subplots")
% tiledlayout(1,3);
% dataaaa = csvread('trajData7_pos_TaskSpace_quintic.csv');
% part5to7(dataaaa)
% title('Acceleration vs Time Data Quintic Task Space Trajectory')

%  data1 = csvread('triangle_0-1.csv')
%  data2 = csvread('triangle_1-2.csv')
%  data3 = csvread('triangle_2-3.csv')
%  
%  
%  disp(data1(end,4)) %01
%  disp(data2(end,4)) %12
%  disp(data3(end,4)) %20

% Sign off 1
% iswearitworks = ik3001([100;0;195])
% fk3001(iswearitworks)
% iswearitworks = ik3001([300;0;0])
% fk3001(iswearitworks)

%part3_jointAngles()

function part3_pvt()
    ikData = csvread('triangle.csv')
    time = ikData(:,4);
    x = ikData(:,1);
    y = ikData(:,2);
    z = ikData(:,3);
    
    plot(time,x,"ro-", 'LineWidth', 1)
    hold on
    plot(time,y,"go-", 'LineWidth', 1)
    plot(time,z,"bo-", 'LineWidth', 1)
    legend("Angle X", "Angle Y", "Angle Z")
    title('Inverse Kinematics - Position vs Time (Interpolated)')
    xlabel('Time in seconds')
    ylabel('Position (mm)')
    hold off
end

function part3_jointAngles()
    ikData = csvread('triangleANG.csv')
    time = ikData(:,4);
    x = ikData(:,1);
    y = ikData(:,2);
    z = ikData(:,3);
    
    plot(time,x,"ro-", 'LineWidth', 1)
    hold on
    plot(time,y,"go-", 'LineWidth', 1)
    plot(time,z,"bo-", 'LineWidth', 1)
    legend("q1", "q2", "q3")
    title('Inverse Kinematics - Joint Angles vs Time (Interpolated)')
    xlabel('Time in seconds')
    ylabel('Joint Angle (deg)')
    hold off
end

function bonus_3d()
data = csvread('BONUS_xyz.csv')
x = data(:,2);
y = data(:,3);
z = data(:,4);
plot3(x,y,z)
hold on
xlim([-150 250])
ylim([-150 250])
zlim([-150 250])
title('Path Followed')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
zlabel('Z Position (mm)')
hold off
end
function part3_3d()
    
    ikDataA = csvread('triangle_0-1.csv')
    ikDataB = csvread('triangle_1-2.csv')
    ikDataC = csvread('triangle_2-3.csv')
    xa = ikDataA(:,1);
    ya = ikDataA(:,2);
    za = ikDataA(:,3);
    
    xb = ikDataB(:,1);
    yb = ikDataB(:,2);
    zb = ikDataB(:,3);
    
    xc = ikDataC(:,1);
    yc = ikDataC(:,2);
    zc = ikDataC(:,3);
    

    plot3(xa, ya, za,"ro");
    hold on
    plot3(xb, yb, zb,"go");
    plot3(xc, yc, zc,"bo");

    title('Path Followed')

    xlabel('X Position (mm)')
    ylabel('Y Position (mm)')
    zlabel('Z Position (mm)')

     legend("Home>Pos1", "Pos1>Pos2 ", "Pos2>Home")

    hold off
end

function part5to7(data)
    xyz_info = [];
    every_seventh = [];
    seven = 0;
    for i = 1:size(data)
        finalPointInfo = fk3001([data(i,2);data(i,3);data(i,4)]);
        xyz_info = [xyz_info;finalPointInfo(1:3,4).'];
        if(mod(seven,14) == 0)
            every_seventh = [every_seventh; data(i,1) xyz_info(i,:)];
        end
        seven = seven + 1;
    end
    x = xyz_info(:,1);
    y = xyz_info(:,2);
    z = xyz_info(:,3);
    time = data (:,1);
    
    %Plotting xyz in respect to time
    nexttile
    plot(time,x,"ro-", 'LineWidth', 1)
    hold on
    plot(time,y,"go-", 'LineWidth', 1)
    plot(time,z,"bo-", 'LineWidth', 1)
    title('Position vs Time Data Quintic Joint Space Trajectory')
    xlabel('Time (s)')
    ylabel('Position (mm)')
    legend("X", "Y", "Z")
    hold off
    
    nexttile
    dx = diff(every_seventh(:,2))./diff(every_seventh(:,1));
    dy = diff(every_seventh(:,3))./diff(every_seventh(:,1));
    dz = diff(every_seventh(:,4))./diff(every_seventh(:,1));
    plot(every_seventh(1:end-1,1),dx,"ro-")
    hold on
    plot(every_seventh(1:end-1,1),dy,"go-")
    plot(every_seventh(1:end-1,1),dz,"bo-")
    title('Velocity vs Time Data Quintic Joint Space Trajectory')
    xlabel('Time (s)')
    ylabel('Velocity (mm/s)')
    legend("X", "Y", "Z")
    hold off
    
    nexttile
    ddx = diff(dx)/diff(every_seventh(:,1));
    ddy = diff(dy)/diff(every_seventh(:,1));
    ddz = diff(dz)/diff(every_seventh(:,1));
    plot(every_seventh(1:end-2,1),ddx(:,1),"ro-")
    hold on
    plot(every_seventh(1:end-2,1),ddy(:,1),"go-")
    plot(every_seventh(1:end-2,1),ddz(:,1),"bo-")
    xlabel('Time (s)')
    ylabel('Acceleration (mm/s²)')
    legend("X", "Y", "Z")
    hold off
    
end

function part6_plot(data)
    xyz_info = [];
    for i = 1:size(data)
        finalPointInfo = fk3001([data(i,2);data(i,3);data(i,4)]);
        xyz_info = [xyz_info;finalPointInfo(1:3,4).'];
    end
    x = xyz_info(:,1);
    y = xyz_info(:,2);
    z = xyz_info(:,3);
    
    plot3(x, y, z,"bo");
    title('Motion 6')
    xlabel('X Position (mm)')
    ylabel('Y Position (mm)')
    zlabel('Z Position (mm)')
    
    xlim([-150 250])
    ylim([-150 250])
    zlim([-150 250])

    legend("Cubic With Task Space", "Cubic With Joint Space", "No Onboard Interpolation")
end

function joint_angles = ik3001(space)
    x = space(1);
    y = space(2);
    z = space(3);
    theta1 = rad2deg(atan2(y,x));
    Px = sqrt(x^2+y^2);
    Py = z - 95;
    L2 = 100;
    L3 = 100;

    alpha = atan2(Py,Px);
    beta_D = ((L2)^2 + Px^2 + Py^2 - (L3)^2)/(2*L2*sqrt(Px^2 + Py^2));
    beta_C = sqrt(1-beta_D^2);
    beta = atan2(beta_C,beta_D);
    theta2 = alpha+beta;
    theta2 = 90-rad2deg(theta2)     
    theta3_D = -((L2)^2 + (L3)^2 - (Px^2 + Py^2))/(2*L2*L3);
    theta3_C = -sqrt(1-theta3_D^2);
    theta3 = atan2(theta3_C,theta3_D);
    theta3 = -90-rad2deg(theta3);

    %Checking for if the angles are valid for the arm
    if(theta2 > -66 && theta2 < 96 && theta3 > -103 && theta3 < 33 && z > 0)
        joint_angles = [theta1; theta2; theta3];
    else
        disp("Did not find a valid position")
        joint_angles = [0; 0; 0];
    end

end

function transform = fk3001(joints)
    joints = deg2rad(joints);
    transform = eye(4);
    %dhTable = [joints(1) L1 0 -90;joints(2)-90 0 L2 0;joints(3)+90 0 l3 0];
    sizeJ = size(joints,1);

    switch sizeJ
        case 1
            x = joints(1);
            zerotwo = [cos(x) 0 -sin(x) 0; sin(x) 0 cos(x) 0; 0 -1 0 95; 0 0 0 1];
            transform = zerotwo;
        case 2   
            x = joints(1);
            y = joints(2);
            zt_1 = [cos(x)* cos((2*y-pi)/2) -cos(x)*sin((2*y-pi)/2) -sin(x) 100*cos(x)*cos((2*y-pi)/2)];
            zt_2 = [sin(x)*cos((2*y-pi)/2) -sin(x)*sin((2*y-pi)/2) cos(x) 100*sin(x)*cos((2*y-pi)/2)];
            zt_3 = [-sin((2*y-pi)/2) -cos((2*y-pi)/2) 0 -100*sin((2*y-pi)/2)+95];
            zt_4 = [0 0 0 1];
            transform = [zt_1; zt_2; zt_3; zt_4];

        case 3
            x = joints(1);
            y = joints(2);
            z = joints(3);
            zt_1 = [cos(x)* cos((2*y-pi)/2) -cos(x)*sin((2*y-pi)/2) -sin(x) 100*cos(x)*cos((2*y-pi)/2)];
            zt_2 = [sin(x)*cos((2*y-pi)/2) -sin(x)*sin((2*y-pi)/2) cos(x) 100*sin(x)*cos((2*y-pi)/2)];
            zt_3 = [-sin((2*y-pi)/2) -cos((2*y-pi)/2) 0 -100*sin((2*y-pi)/2)+95];
            zt_4 = [0 0 0 1];
            part1 = [zt_1; zt_2; zt_3; zt_4];
            part2  = [cos(z+(pi/2)) -sin(z+(pi/2)) 0 100*cos(z+pi/2); sin(z+(pi/2)) cos(z+(pi/2)) 0 100*sin(z+pi/2); 0 0 1 0; 0 0 0 1];
            transform = part1 * part2;
    end
end