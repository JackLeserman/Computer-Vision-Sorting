partFour()

function partFour()
%unscaled
part4Graph = csvread('homePosTransform(Part4).csv')
%perfectPoint = [100; 0; 195]
meanX = mean(part4Graph(1,:));
meanY = mean(part4Graph(2,:));
meanZ = mean(part4Graph(3,:));
avgTipPos = [meanX; meanY; meanZ]
perfectX = 100;
perfectY = 0;
perfectZ = 195;

distances = zeros(1,10);
for i = 1:10
    checkPoint = part4Graph(:,i)'
    xAct = checkPoint(1)
    yAct = checkPoint(2)
    zAct = checkPoint(3)
    dist = ((xAct-perfectX)^2 + (yAct-perfectY)^2 + (zAct-perfectZ)^2)
    distances(i) = dist;
end
rmsData = sqrt(mean(distances))

stem3(part4Graph(1,:), part4Graph(2,:), part4Graph(3,:))
%xlim([100 103])
%ylim([0 3])
%zlim([193 196])
title('Tip Position (mm)')

xlabel('X Position (mm)')
ylabel('Y Position (mm)')
zlabel('Z Position (mm)')
disp(rmsData)

disp(mean(part4Graph(1,:)))
disp(mean(part4Graph(2,:)))
disp(mean(part4Graph(3,:)))


end

function partEightA(robot)
    part8graph = csvread('part8_3.csv')
    currPosAng = part8graph(:,1:3);
    time = part8graph(:,4);
    arm0 = part8graph(:,5:7,:);
    arm1 = part8graph(:,8:10,:);
    arm2 = part8graph(:,11:13,:);
    arm3 = part8graph(:,14:16,:);
    arm4 = part8graph(:,17:19);
    
    plot(time,currPosAng(:,1),"ro-", 'LineWidth', 2)
    hold on
    plot(time,currPosAng(:,2),"gs-", 'LineWidth', 2)
    plot(time,currPosAng(:,3),"b*-", 'LineWidth', 2)
    legend("Angle X", "Angle Y", "Angle Z")
    title('Joint Angle vs Time (Pos 3)')
    xlabel('Time in seconds')
    ylabel('Position (Degrees)')
    hold off
end

function partEightB(robot)
    part8graph = csvread('part8_1.csv')
    currPosAng = part8graph(:,1:3);
    time = part8graph(:,4);
    arm0 = part8graph(:,5:7,:);
    arm1 = part8graph(:,8:10,:);
    arm2 = part8graph(:,11:13,:);
    arm3 = part8graph(:,14:16,:);
    arm4 = part8graph(:,17:19);
    
    plot(time,arm4(:,1),"ro-", 'LineWidth', 1)
    hold on
    plot(time,arm4(:,3),"b*-", 'LineWidth', 1)
    legend("Pos X", "Pos Z")
     title('Tip Position vs Time (Pos 1)')
    xlabel('Time in seconds')
    ylabel('Position (mm)')
    hold off
end

function partEightC(robot)
    part8grapha = csvread('part8_3a.csv')
    part8graphb = csvread('part8_3b.csv')
    part8graphc = csvread('part8_3c.csv')

    arm4a = part8grapha(:,17:19);
    arm4b = part8graphb(:,17:19);
    arm4c = part8graphc(:,17:19);
    
    plot(arm4a(:,1),arm4a(:,3),"ro-", 'LineWidth', 1)
    hold on
    plot(arm4b(:,1),arm4b(:,3),"bo-", 'LineWidth', 1)
    plot(arm4c(:,1),arm4c(:,3),"go-", 'LineWidth', 1)
    legend("Movement 1", "Movement 2", "Movement 3")
    title('Tip Position vs Time (Pos 3)')
    xlabel('PosX (mm)')
    ylabel('PosZ (mm)')
    hold off
end

function partEightD(robot)
    part8grapha = csvread('part8_3a.csv')
    part8graphb = csvread('part8_3b.csv')
    part8graphc = csvread('part8_3c.csv')

    arm4a = part8grapha(:,17:19);
    arm4b = part8graphb(:,17:19);
    arm4c = part8graphc(:,17:19);
    
    plot(arm4a(:,1),arm4a(:,2),"ro-", 'LineWidth', 1)
    hold on
    plot(arm4b(:,1),arm4b(:,2),"bo-", 'LineWidth', 1)
    plot(arm4c(:,1),arm4c(:,2),"go-", 'LineWidth', 1)
    legend("Movement 1", "Movement 2", "Movement 3")
    title('Tip Position vs Time (Pos 3)')
    xlabel('PosX (mm)')
    ylabel('PosY (mm)')
    hold off
end
    
    


