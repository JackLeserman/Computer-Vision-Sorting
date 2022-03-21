%The following will satisfy the graphing requirement
%in part 4...

%-------------GLOBALS--------------

%Put this in a try statement since there may be problems with the file
    part4Interpolated1 = csvread('jointimeNOTInterpolated1.csv')
    part4Interpolated2 = csvread('jointimeNOTInterpolated2.csv')
    part4Interpolated3 = csvread('jointimeNOTInterpolated3.csv')
    part4UnInterpolated1 = csvread('jointimeNOTInterpolated1.csv')
    part4UnInterpolated2 = csvread('jointimeNOTInterpolated2.csv')
    part4UnInterpolated3 = csvread('jointimeNOTInterpolated3.csv')

    part5pos1Interpolated = csvread('position1Interpolated.csv')
    part5pos2Interpolated = csvread('position2Interpolated.csv')
    part5pos3Interpolated = csvread('position3Interpolated.csv')
    part5pos4Interpolated = csvread('position4Interpolated.csv')

    part5pos1UnInterpolated = csvread('position1UnInterpolated.csv')
    part5pos2UnInterpolated = csvread('position2UnInterpolated.csv')
    part5pos3UnInterpolated = csvread('position3UnInterpolated.csv')
    part5pos4UnInterpolated = csvread('position4UnInterpolated.csv')

% % %-----------------------------------------
% % %-------------------4---------------------
% % %-----------------------------------------
% 
p4I1 = part4Interpolated1(:,1)
p4I2 = part4Interpolated1(:,2)
p4I3 = part4Interpolated1(:,3)
p4I1shift = p4I1 + 90;
p4I2shift = p4I2 + 90;
p4I3shift = p4I3 + 90;

plot(part4Interpolated1(:,4)-part4Interpolated1(1,4),part4Interpolated1(:,1),"b*-", 'LineWidth', 2)
hold on
title('Motion  Trajectory Without Interpolation')
xlabel('Time in seconds')
ylabel('Position')

plot(part4Interpolated1(:,4)-part4Interpolated1(1,4),part4Interpolated1(:,2),"ro-", 'LineWidth', 2)
plot(part4Interpolated1(:,4)-part4Interpolated1(1,4),part4Interpolated1(:,3),"gs-", 'LineWidth', 2)
plot(part4Interpolated2(:,4)-part4Interpolated2(1,4),part4Interpolated2(:,1),"b*-", 'LineWidth', 2)
plot(part4Interpolated2(:,4)-part4Interpolated2(1,4),part4Interpolated2(:,2),"r+-", 'LineWidth', 2)
plot(part4Interpolated2(:,4)-part4Interpolated2(1,4),part4Interpolated2(:,3),"gx-", 'LineWidth', 2)
plot(part4Interpolated3(:,4)-part4Interpolated3(1,4),part4Interpolated3(:,1),"b^-", 'LineWidth', 2)
plot(part4Interpolated3(:,4)-part4Interpolated3(1,4),part4Interpolated3(:,2),"rp-", 'LineWidth', 2)
plot(part4Interpolated3(:,4)-part4Interpolated3(1,4),part4Interpolated3(:,3),"gh-", 'LineWidth', 2)
legend("Motor1 Pos I", "Motor2 Pos I", "Motor3 Pos I","Motor1 Pos II", "Motor2 Pos II", "Motor3 Pos II","Motor1 Pos III", "Motor2 Pos III", "Motor3 Pos III")
hold off

%-----------------------------------------
%--------------------5--------------------
%-----------------------------------------

%Part 1 of Part 5
    
        
%     plot(part5pos1Interpolated(:,4)-part5pos1Interpolated(1,4),part5pos1Interpolated(:,1),"ro-",'LineWidth',2)
%     hold on
%     title('Movement 1 Interpolated')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     plot(part5pos1Interpolated(:,4)-part5pos1Interpolated(1,4),part5pos1Interpolated(:,2),"b*-",'LineWidth',2)
%     plot(part5pos1Interpolated(:,4)-part5pos1Interpolated(1,4),part5pos1Interpolated(:,3),"gs-",'LineWidth',2)
%     hold off

%     plot(part5pos2Interpolated(:,4)-part5pos2Interpolated(1,4),part5pos2Interpolated(:,1),"ro-",'LineWidth',2)
%     hold on
%     title('Movement 2 Interpolated')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     plot(part5pos2Interpolated(:,4)-part5pos2Interpolated(1,4),part5pos2Interpolated(:,2),"b*-",'LineWidth',2)
%     plot(part5pos2Interpolated(:,4)-part5pos2Interpolated(1,4),part5pos2Interpolated(:,3),"gs-",'LineWidth',2)
%     hold off

%     plot(part5pos3Interpolated(:,4)-part5pos3Interpolated(1,4),part5pos3Interpolated(:,1),"ro-",'LineWidth',2)
%     hold on
%     title('Movement 3 Interpolated')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     plot(part5pos3Interpolated(:,4)-part5pos3Interpolated(1,4),part5pos3Interpolated(:,2),"b*-",'LineWidth',2)
%     plot(part5pos3Interpolated(:,4)-part5pos3Interpolated(1,4),part5pos3Interpolated(:,3),"gs-",'LineWidth',2)
%     hold off

%     plot(part5pos4Interpolated(:,4)-part5pos4Interpolated(1,4),part5pos4Interpolated(:,1),"ro-",'LineWidth',2)
%     hold on
%     title('Movement 4 Interpolated')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     plot(part5pos4Interpolated(:,4)-part5pos4Interpolated(1,4),part5pos4Interpolated(:,2),"b*-",'LineWidth',2)
%     plot(part5pos4Interpolated(:,4)-part5pos4Interpolated(1,4),part5pos4Interpolated(:,3),"gs-",'LineWidth',2)
%     hold off
    

%Part 3 of Part 5

    
%     plot(part5pos1Interpolated(:,4)-part5pos1Interpolated(1,4),part5pos1Interpolated(:,1),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos1UnInterpolated(:,4)-part5pos1UnInterpolated(1,4),part5pos1UnInterpolated(:,1),"b*-",'LineWidth',2)
%     title('Movement 1 X')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
%     
    
%     %nexttile

%     plot(part5pos1Interpolated(:,4)-part5pos1Interpolated(1,4),part5pos1Interpolated(:,2),"ro-",'LineWidth',2)
%     hold on
%     title('Movement 1 Y')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     plot(part5pos1UnInterpolated(:,4)-part5pos1UnInterpolated(1,4),part5pos1UnInterpolated(:,2),"b*-",'LineWidth',2)
%     legend("Interpolated", "Uninterpolated")
%     hold off
%     %nexttile
%     plot(part5pos1Interpolated(:,4)-part5pos1Interpolated(1,4),part5pos1Interpolated(:,3),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos1UnInterpolated(:,4)-part5pos1UnInterpolated(1,4),part5pos1UnInterpolated(:,3),"b*-",'LineWidth',2)
%     title('Movement 1 Z')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
% %     %nexttile
    
%     %Position 2
%     plot(part5pos2Interpolated(:,4)-part5pos2Interpolated(1,4),part5pos2Interpolated(:,1),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos2UnInterpolated(:,4)-part5pos2UnInterpolated(1,4),part5pos2UnInterpolated(:,1),"b*-",'LineWidth',2)
%     title('Movement 2 X')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
%     %nexttile
%     plot(part5pos2Interpolated(:,4)-part5pos2Interpolated(1,4),part5pos2Interpolated(:,2),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos2UnInterpolated(:,4)-part5pos2UnInterpolated(1,4),part5pos2UnInterpolated(:,2),"b*-",'LineWidth',2)
%     title('Movement 2 Y')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
%     %nexttile
%     plot(part5pos2Interpolated(:,4)-part5pos2Interpolated(1,4),part5pos2Interpolated(:,3),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos2UnInterpolated(:,4)-part5pos2UnInterpolated(1,4),part5pos2UnInterpolated(:,3),"b*-",'LineWidth',2)
%     title('Movement 2 Z')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
%     %nexttile
%     
%     %Position 3
%     plot(part5pos3Interpolated(:,4)-part5pos3Interpolated(1,4),part5pos3Interpolated(:,1),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos3UnInterpolated(:,4)-part5pos3UnInterpolated(1,4),part5pos3UnInterpolated(:,1),"b*-",'LineWidth',2)
%     title('Movement 3 X')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
% % 
%     plot(part5pos3Interpolated(:,4)-part5pos3Interpolated(1,4),part5pos3Interpolated(:,2),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos3UnInterpolated(:,4)-part5pos3UnInterpolated(1,4),part5pos3UnInterpolated(:,2),"b*-",'LineWidth',2)
%     title('Movement 3 Y')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
% 
%     plot(part5pos3Interpolated(:,4)-part5pos3Interpolated(1,4),part5pos3Interpolated(:,3),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos3UnInterpolated(:,4)-part5pos3UnInterpolated(1,4),part5pos3UnInterpolated(:,3),"b*-",'LineWidth',2)
%     title('Movement 3 Y')
%     xlabel('Time in seconds')
%     ylabel('Position')
%     legend("Interpolated", "Uninterpolated")
%     hold off
% %     
%     
% %   %Position 4
%     plot(part5pos4Interpolated(:,4)-part5pos4Interpolated(1,4),part5pos4Interpolated(:,1),"ro-",'LineWidth',2)
% %     hold on
% %     plot(part5pos4UnInterpolated(:,4)-part5pos4UnInterpolated(1,4),part5pos4UnInterpolated(:,1),"b*-",'LineWidth',2)
%        title('Movement 4 X')
%        xlabel('Time in seconds')
%        ylabel('Position')
%        legend("Interpolated", "Uninterpolated")
%     hold off
%     hold off
%     %nexttile
%     plot(part5pos4Interpolated(:,4)-part5pos4Interpolated(1,4),part5pos4Interpolated(:,2),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos4UnInterpolated(:,4)-part5pos4UnInterpolated(1,4),part5pos4UnInterpolated(:,2),"b*-",'LineWidth',2)
%        title('Movement 4 Y')
%        xlabel('Time in seconds')
%        ylabel('Position')
%        legend("Interpolated", "Uninterpolated")
%     hold off
%     %nexttile
%     plot(part5pos4Interpolated(:,4)-part5pos4Interpolated(1,4),part5pos4Interpolated(:,3),"ro-",'LineWidth',2)
%     hold on
%     plot(part5pos4UnInterpolated(:,4)-part5pos4UnInterpolated(1,4),part5pos4UnInterpolated(:,3),"b*-",'LineWidth',2)
%     title('Movement 4 Z')
%        xlabel('Time in seconds')
%        ylabel('Position')
%        legend("Interpolated", "Uninterpolated")
%     hold off


% plot(data_noint1(:,4)-data_noint1(1,4),data_noint1(:,1),"b*-", 'LineWidth', 2)
% hold on
% title('Motion  Trajectory w/o Interpolation')
% xlabel('Time in seconds')
% ylabel('Position')
% plot(data_noint1(:,4)-data_noint1(1,4),data_noint1(:,2),"ro-", 'LineWidth', 2)
% plot(data_noint1(:,4)-data_noint1(1,4),data_noint1(:,3),"gs-", 'LineWidth', 2)

%legend("Motor1 Pos I", "Motor2 Pos I", "Motor3 Pos I","Motor1 Pos II", "Motor2 Pos II", "Motor3 Pos II","Motor1 Pos III", "Motor2 Pos III", "Motor3 Pos III")

