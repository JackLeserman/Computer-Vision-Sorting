clear
%part5_linear();
%part5_angular();
%part5_scalar();
estop_3d();
%estop_DET();

function part5_linear()
    data = csvread('jacobMov.csv')
    time = data(:,1);
    x = data(:,2);
    y = data(:,3);
    z = data(:,4);
    
    plot(time,x,"ro-")
    hold on
    plot(time,y,"go-")
    plot(time,z,"bo-")
    legend("Vel X", "Vel Y", "Vel Z")
    title('Velocity vs Time')
    xlabel('Time in seconds')
    ylabel('Linear Velocity (mm/s)')
    hold off
end


function part5_scalar()
    data = csvread('jacobMov.csv')
    time = data(:,1);
    x = data(:,2);
    y = data(:,3);
    z = data(:,4);
    scalar = sqrt(x.^2 + y.^2 + z.^2);
    
    
    plot(time,scalar,"ro")
    legend("Vel Scalar")
    title('Velocity vs Time')
    xlabel('Time in seconds')
    ylabel('Scalar Velocity (mm/s)')
    hold off
end

function estop_DET()
data = csvread('eStopDET2.csv')
t = data(:,1);
jp = data(:,2);
plot(t,jp,"ro-");
hold on
title('Det jp vs Time')
xlabel('Time (s)')
ylabel('Det jp')
hold off
end

function estop_3d()
data = csvread('eStopXYZ2.csv')
x = data(:,1);
y = data(:,2);
z = data(:,3);
plot3(x,y,z)
hold on
xlim([0 150])
ylim([0 150])
zlim([0 150])
title('Path Followed')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
zlabel('Z Position (mm)')
hold off
end

function part5_angular()
    data = csvread('jacobMov(noPlot).csv')
    time = data(:,1);
    w1 = data(:,5);
    w2 = data(:,6);
    w3 = data(:,7);
    
    plot(time,w1,"ro")
    hold on
    plot(time,w2,"go")
    plot(time,w3,"bo")
    legend("Vel X", "Vel Y", "Vel Z")
    title('Velocity vs Time')
    xlabel('Time in seconds')
    ylabel('Angular Velocity (deg/s)')
    hold off
end
