classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962

        %setPoint globals
        goal = zeros(1,4);
        L0 = 55;
        L1 = 40;
        L2 = 100;
        L3 = 100;

    end
    
    methods
        %Joint Space Robot Control
        

        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
             self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        

        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end
        
        
        %Lab 1 Functions
        %Takes a 1x3 array of joint values in degrees to be sent directly
        %to the actuators and bypasses interpolation
        
        function servo_jp(self, arr)
            packet = zeros(15, 1, 'single');
            SERV_ID = 1848;
            packet(2) = 0;
            packet(3) = arr(1);
            packet(4) = arr(2);% Second link to 0
            packet(5) = arr(3);% Third link to 0

            
            %this should work to set goal
            self.goal(1)=packet(1);
            self.goal(2:4)=packet(3:5);
            
            self.write(SERV_ID, packet); 
        end
        
        %Takes a 1x3 array of joint values and an interpolation time in ms
        %to get there
        function interpolate_jp(self, arr, time)

            packet = zeros(15, 1, 'single');
            SERV_ID = 1848;
            packet(1) = time;
            packet(2) = 0;
            packet(3) = arr(1);
            packet(4) = arr(2);% Second link to 0
            packet(5) = arr(3);% Third link to 0
            
            %this should work to set goal
            self.goal(1)=packet(1);
            
            
            self.write(SERV_ID, packet);

        end

        %Retyrbs a 2x3 array that contains current joint positions in
        %degrees (1st row), and or current joint velocities (2nd row)
        function return_arr = measured_js(self, GETPOS, GETVEL)
            return_arr = zeros(2,3);
            if(GETPOS)
                %Makes first row the current joint positions in degrees
                SERV_ID = 1910;
                packet = self.read(SERV_ID);
                return_arr(1,1) = packet(3);
                return_arr (1,2) = packet(5);
                return_arr(1,3) = packet(7);
                
            end
            if(GETVEL)
                SERV_ID = 1822;
                packet = self.read(SERV_ID);
                return_arr(2,1) = packet(3);
                return_arr(2,2) = packet(6);
                return_arr(2,3) = packet(9);     
            end
        end
        
        
        %Returns a 1x3 array that contains the current joint set point
        %positions in degrees. If interpolation is being used and you
        %requested during motion, will return the current immediate set
        %point
        function matrix = setpoint_js(self)
            %add stuff for global interpolation
            matrix = zeros(1,3);
            id = 1910;
            packet = self.read(id);
            matrix(1,1) = packet(2);
            matrix(1,2) = packet(4);
            matrix(1,3) = packet(6);
            
        end
        
        %Returns a 1x3 array that contains the end of motion joint setpoint
        %positions in degrees.
        %Should be stored directly in your robot object, does not need to
        %be requested from the controller

        function ret_goal = goal_js(self)
            ret_goal = self.goal(2:4);
        end
        
        
        %Lab 2 Functions
        
        
        function transform = trotz(theta) 
            transform = eye(4); %4x4 identity matrix
            disp(transform);
            transform(1,1) = cosd(theta);
            transform(2,1) = -sind(theta);
            transform(1,2) = sind(theta);
            transform(2,2) = cosd(theta);
        end

        %transformation matrix for rotate about y by angle theta
        function transform = troty(theta) 
            transform = eye(4); %4x4 identity matrix
            transform(1,1) = cosd(theta);
            transform(1,3) = sind(theta);
            transform(3,1) = -sind(theta);
            transform(3,3) = cosd(theta);
        end
        
        %transformation matrix for rotate about x by angle theta
        function transform = trotx(theta) 
            transform = eye(4); %4x4 identity matrix
            transform(2,2) = cosd(theta);
            transform(2,3) = -sind(theta);
            transform(3,2) = sind(theta);
            transform(3,3) = cosd(theta);
        end
        
        %Takes in a 1x4 array corresponding to a row of the DH parameter
        %table for a given link, then generates the associated intermediate
        %transformation and returns a corresponding symbolic 4x4
        %homogeneous transformation matrix
        function transform = dh2mat(self,params)

            theta = params(1);

            d = params(2);
            a = params(3);
            alpha = params (4);

            r1 = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta)];
            r2 = [sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta)];
            r3 = [0 sind(alpha) cosd(alpha) d];
            r4 = [0 0 0 1];
            transform = [r1; r2; r3; r4];
            
        end
        
        %Using dh2mat, takes in an nx4 array corresponding to the number of
        %rows of the full DH parameter table, then generates a
        %corresponding symbolic 4xx4 homogeneous transformation matrix for
        %the composite transformation.
        function transform = dh2fk(self,params)
                numRows = size(params,1); %number of rows
                transform = eye(4); %4x4 identity matrix
                for i = 1:numRows
                    transform = transform*self.dh2mat(params(i,:)); 
                    disp(transform)
                end

        end
        
        %Takes n joint configuration as inputs in the form of a nx1 vector,
        %and returns a 4x4 homogeneous transformation matrix representing
        %the position and orientation of the ti frame with repect to the
        %base frame
        
        function transform = fk3001_wMath(self,joints)
            L0 = self.L0;
            L1 = self.L1;
            L2 = self.L2;
            l3 = self.L3;
            
            transform = eye(4);
            transform(3,4) = L0; %transfer 0->1
            %dhTable = [joints(1) L1 0 -90;joints(2)-90 0 L2 0;joints(3)+90 0 l3 0];
            sizeJ = size(joints,1);
            switch sizeJ
                case 1
                    dhTable = [joints(1) L1 0 -90];
                    transform = self.dh2fk(dhTable);
                    disp("One Joint")
                case 2
                    dhTable = [joints(1) L1 0 -90;joints(2)-90 0 L2 0];
                    transform = self.dh2fk(dhTable);
                    disp("Two Joint")
                case 3
                    
                    dhTable = [joints(1) L1 0 -90;joints(2)-90 0 L2 0;joints(3)+90 0 l3 0];
                    transform = self.dh2fk(dhTable);
                    disp("Three Joint")
            end
        end
         
        %TAKES IN COLUMN VECTOR: DONT FUCK UP!!!!!!!
        function transform = fk3001(self,joints)
            joints = deg2rad(joints);
            transform = eye(4);
            transform(3,4) = self.L0; %transfer 0->1
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
        
        function transform = fk3001ZAxis(self,joints)
            joints = deg2rad(joints);
            
            transform = eye(4);
            transform(3,4) = self.L0; %transfer 0->1
            %dhTable = [joints(1) L1 0 -90;joints(2)-90 0 L2 0;joints(3)+90 0 l3 0];
            sizeJ = size(joints,1);
            zShift = [1 0 0 0;0 1 0 0;0 0 1 30;0 0 0 1];
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
            transform = transform * zShift;
        end
        
        %TAKES IN COLUMN VECTOR: DONT FUCK UP!!!!!!!
        function transform = fk3001XAxis(self,joints)
            joints = deg2rad(joints);
            
            transform = eye(4);
            transform(3,4) = self.L0; %transfer 0->1
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
            xShift = [1 0 0 30; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            transform = transform * xShift;
            
            
        end
        
        %Takes data from measured_js() and returns a 4x4 homogeneous
        %transformation matrix based upon the current joint set point
        %positions in degrees
        function transform = measured_cp(self)
            measuredJS = self.measured_js(1,0);
            ang = deg2rad(measuredJS(1,:));
            colAng = transpose(ang);
            transform = self.fk3001(colAng);
        end
        
        %Takes data from setpoint_js() and returns a 4x4 homogeneous
        %transformation matrix based upon the current joint set point
        %positions in degrees. If interpolation is being used and yo
        %request during motion, it will return the current intermediate set
        %point
        function transform = setpoint_cp(self)
            setpoint = self.setpoint_js();
            ang = deg2rad(setpoint);
            transform = self.fk3001(ang);
        end
        
        %Takes data from goal_js and returns a 4x4 homogeneous
        %transformation matrix based upon the commanded end of motion joint
        %set positions in degrees
        function transform = goal_cp(self)
            goal = self.goal_js();
            ang = deg2rad(goal);
            transform = self.fk3001(ang);
            disp("3")
            disp(transform)
        end
        
        
        %Takes a 3x1 array of joint values and plots a stick model of the arm
        %showing all frames, joints, and links
        %For the dimension of each link, use global variables L0:L3, and use
        %orientations of the corresponding rotation matrices to determine the
        %direction of each of the unit vectors for each coordinate frame
        %Use functions generated in part 2 for this part wherever possible
        function plot_arm(self,joint_values)
            armPos = [0 0 0; 0 0 55];
            xPos = [30,0,0;30,0,55];          
            zPos = [0 0 30; 0 0 85];
            joint_values = transpose(joint_values);
            zShift = [1 0 0 0;0 1 0 0;0 0 1 30;0 0 0 1];
            xShift = [1 0 0 30; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for i = 1:3
                nextJoint = joint_values(1:i,1);
                newPos = self.fk3001(nextJoint);
                newXPos = self.fk3001(nextJoint) * xShift;
                newZPos = self.fk3001(nextJoint) * zShift;
                newPoint = newPos(1:3,4).'; % Gets the last column,
                newZPoint = newZPos(1:3,4).';
                newXPoint = newXPos(1:3,4).';
                %Then converts it into a row to put into the armPos
                zPos = [zPos;newZPoint];
                armPos = [armPos;newPoint];
                xPos = [xPos;newXPoint];
            end
            
            unga = plot3(armPos(:,1),armPos(:,2),armPos(:,3),...
                '-ok','LineWidth',2,'MarkerSize',6)
            hold on
            bunga = plot3([zPos(1,1) armPos(1,1)],[zPos(1,2) armPos(1,2)],[zPos(1,3) armPos(1,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(2,1) armPos(2,1)],[zPos(2,2) armPos(2,2)],[zPos(2,3) armPos(2,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(3,1) armPos(3,1)],[zPos(3,2) armPos(3,2)],[zPos(3,3) armPos(3,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(4,1) armPos(4,1)],[zPos(4,2) armPos(4,2)],[zPos(4,3) armPos(4,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(5,1) armPos(5,1)],[zPos(5,2) armPos(5,2)],[zPos(5,3) armPos(5,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            chalupa = plot3([xPos(1,1) armPos(1,1)],[xPos(1,2) armPos(1,2)],[xPos(1,3) armPos(1,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(2,1) armPos(2,1)],[xPos(2,2) armPos(2,2)],[xPos(2,3) armPos(2,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(3,1) armPos(3,1)],[xPos(3,2) armPos(3,2)],[xPos(3,3) armPos(3,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(4,1) armPos(4,1)],[xPos(4,2) armPos(4,2)],[xPos(4,3) armPos(4,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(5,1) armPos(5,1)],[xPos(5,2) armPos(5,2)],[xPos(5,3) armPos(5,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
                
            title('RBE 3001 Robot Arm')
            xlabel('X Axis (mm)');
            ylabel('Y Axis (mm)');
            zlabel('Z Axis (mm)');
            legend([unga,bunga,chalupa],'Robot Arm','zAxis of Joint','xAxis of Joint')
            xlim([-220 220])
            ylim([-220 220])
            zlim([0 300])
            
            hold off
            pause(0.2)
        end
          
        
        %Takes a 3x1 array of joint values and plots a stick model of the arm
        %showing all frames, joints, and links
        %Returns the arm joint data for all 5 points
        %For the dimension of each link, use global variables L0:L3, and use
        %orientations of the corresponding rotation matrices to determine the
        %direction of each of the unit vectors for each coordinate frame
        %Use functions generated in part 2 for this part wherever possible
        function data = arm_data(self,joint_values)
            data = []
            armPos = [0 0 0; 0 0 55];
            xPos = [30,0,0;30,0,55];          
            zPos = [0 0 30; 0 0 85];
            zShift = [1 0 0 0;0 1 0 0;0 0 1 30;0 0 0 1];
            xShift = [1 0 0 30; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for i = 1:3
                nextJoint = joint_values(1:i,1);
                newPos = self.fk3001(nextJoint);
                newXPos = self.fk3001(nextJoint) * xShift;
                newZPos = self.fk3001(nextJoint) * zShift;
                newPoint = newPos(1:3,4).'; % Gets the last column,
                newZPoint = newZPos(1:3,4).';
                newXPoint = newXPos(1:3,4).';
                %Then converts it into a row to put into the armPos
                zPos = [zPos;newZPoint];
                armPos = [armPos;newPoint];
                xPos = [xPos;newXPoint];
            end
            hold off
            data = armPos;
        end
        
        
        function workspace(self)
            points = [];
            for i = -143:4:143
                for j = -66:4:96 % Need to find min and max for joint 2
                    for k = -103:4:33 % Need to find min and max for joint 3
                        pos = self.fk3001([i;j;k]);
                        newPoint = pos(1:3,4).';
                        if(newPoint(3)>0)
                            points = [points;newPoint]; 
                        end
                    end
                end
                
            end
            disp(points)
            final = boundary(points,1);
            trisurf(final,points(:,1),points(:,2),points(:,3),'FaceColor','red','FaceAlpha',0.1)
        end
        
        
        function joint_angles = ik3001(self, space)
            try
            x = space(1);
            y = space(2);
            z = space(3);
            theta1 = rad2deg(atan2(y,x));
            Px = sqrt(x^2+y^2);
            Py = z - 95;
            
            alpha = atan2(Py,Px);
            beta_D = ((self.L2)^2 + Px^2 + Py^2 - (self.L3)^2)/(2*self.L2*sqrt(Px^2 + Py^2));
            beta_C = sqrt(1-beta_D^2);
            beta = atan2(beta_C,beta_D);
            theta2 = alpha+beta;
            theta2 = 90-rad2deg(theta2);     
            theta3_D = -((self.L2)^2 + (self.L3)^2 - (Px^2 + Py^2))/(2*self.L2*self.L3);
            theta3_C = -sqrt(1-theta3_D^2);
            theta3 = atan2(theta3_C,theta3_D);
            theta3 = -90-rad2deg(theta3);
            
            joint_angles = [theta1; theta2; theta3];
            
%             %Checking for if the angles are valid for the arm
%             if(theta2 > -66 && theta2 < 96 && theta3 > -33 && theta3 < 103 && z > 0)
%                 joint_angles = [theta1; theta2; theta3];
%             else
%                 disp("Did not find a valid position")
%                 joint_angles = [0; 0; 0];
%             end
            catch err
                disp("Code breaky since pain and suffering and its probably not a valid position")
                disp(err)
           end
        end
       
        % trajectory should take (can be xyz or theta1,2,3, 
        % and also whether or not it runs as a
        % task_space or joint space trajectory
        % Will run a while loop which calculates the current joint poses
        % based on the trajectory coefficients and current time, and
        % commands the robot to go to these with server_jp(), also saves
        % time and joint angle data to a matrix to plot later
        % Returns the saved matrix data
        % Matrix_data returns as
        % [time joint1 joint2 joint3]
        % Task space is true if you want it to run in task_space, false if
        % joint space
        function matrix_data = run_trajectory(self,trajectory_coefficients,motion_time,task_space)
            %data = ["Time" "Theta1 Calculated" "Theta2 Calculated" "Theta3 Calculated" "Actual Theta1" "Actual Theta2" "Actual Theta3"];
            data = []
            tic;
            start = tic;
            [go_die,replace_size] = size(trajectory_coefficients)
            joint1_traj = zeros(1,6);
            joint2_traj = zeros(1,6);
            joint3_traj = zeros(1,6);
            joint1_traj(1,1:replace_size) = trajectory_coefficients(1,:);
            joint2_traj(1,1:replace_size) = trajectory_coefficients(2,:);
            joint3_traj(1,1:replace_size) = trajectory_coefficients(3,:);
            toc;
            t = toc;
            while t < motion_time
                t = toc;
                actual_pos = self.measured_js(1,0);
                if task_space
                    curr_pos = self.measured_js(1,1);
                    pointx = joint1_traj(1) + joint1_traj(2)*t + joint1_traj(3)*t^2 + joint1_traj(4)*t^3 + joint1_traj(5)*t^4+joint1_traj(6)*t^5;
                    pointy = joint2_traj(1) + joint2_traj(2)*t + joint2_traj(3)*t^2 + joint2_traj(4)*t^3 + joint2_traj(5)*t^4+joint2_traj(6)*t^5;
                    pointz = joint3_traj(1) + joint3_traj(2)*t + joint3_traj(3)*t^2 + joint3_traj(4)*t^3 + joint3_traj(5)*t^4+joint3_traj(6)*t^5;
                    angles = self.ik3001([pointx; pointy; pointz]);
                    self.servo_jp(transpose(angles));
%                   data = [data; t actual_pos(1,:)];
                    
                else
                    curr_pos = self.measured_js(1,0);
                    theta1 = joint1_traj(1) + joint1_traj(2)*t + joint1_traj(3)*t.^2 + joint1_traj(4)*t.^3 + joint1_traj(5)*t.^4+joint1_traj(6)*t.^5;
                    theta2 = joint2_traj(1) + joint2_traj(2)*t + joint2_traj(3)*t.^2 + joint2_traj(4)*t.^3 + joint2_traj(5)*t^4+joint2_traj(6)*t^5;
                    theta3 = joint3_traj(1) + joint3_traj(2)*t + joint3_traj(3)*t.^2 + joint3_traj(4)*t.^3 + joint3_traj(5)*t^4+joint3_traj(6)*t^5;
                    self.servo_jp([theta1 theta2 theta3]);
                    %data = [data; t theta1 theta2 theta3 actual_pos(1,:)];;
                end
                pause(0.01);
                end
            disp("loop broken")
            matrix_data = data;
        end
        
        function plot_armNODELAY(self,joint_values)
            armPos = [0 0 0; 0 0 55];
            xPos = [30,0,0;30,0,55];          
            zPos = [0 0 30; 0 0 85];
            joint_values = transpose(joint_values);
            zShift = [1 0 0 0;0 1 0 0;0 0 1 30;0 0 0 1];
            xShift = [1 0 0 30; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for i = 1:3
                nextJoint = joint_values(1:i,1);
                newPos = self.fk3001(nextJoint);
                newXPos = self.fk3001(nextJoint) * xShift;
                newZPos = self.fk3001(nextJoint) * zShift;
                newPoint = newPos(1:3,4).'; % Gets the last column,
                newZPoint = newZPos(1:3,4).';
                newXPoint = newXPos(1:3,4).';
                %Then converts it into a row to put into the armPos
                zPos = [zPos;newZPoint];
                armPos = [armPos;newPoint];
                xPos = [xPos;newXPoint];
            end
            
            unga = plot3(armPos(:,1),armPos(:,2),armPos(:,3),...
                '-ok','LineWidth',2,'MarkerSize',6);
            hold on
            bunga = plot3([zPos(1,1) armPos(1,1)],[zPos(1,2) armPos(1,2)],[zPos(1,3) armPos(1,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(2,1) armPos(2,1)],[zPos(2,2) armPos(2,2)],[zPos(2,3) armPos(2,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(3,1) armPos(3,1)],[zPos(3,2) armPos(3,2)],[zPos(3,3) armPos(3,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(4,1) armPos(4,1)],[zPos(4,2) armPos(4,2)],[zPos(4,3) armPos(4,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            plot3([zPos(5,1) armPos(5,1)],[zPos(5,2) armPos(5,2)],[zPos(5,3) armPos(5,3)],...
                '-.b','LineWidth',2,'MarkerSize',6);
            chalupa = plot3([xPos(1,1) armPos(1,1)],[xPos(1,2) armPos(1,2)],[xPos(1,3) armPos(1,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(2,1) armPos(2,1)],[xPos(2,2) armPos(2,2)],[xPos(2,3) armPos(2,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(3,1) armPos(3,1)],[xPos(3,2) armPos(3,2)],[xPos(3,3) armPos(3,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(4,1) armPos(4,1)],[xPos(4,2) armPos(4,2)],[xPos(4,3) armPos(4,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            plot3([xPos(5,1) armPos(5,1)],[xPos(5,2) armPos(5,2)],[xPos(5,3) armPos(5,3)],...
                '-.g','LineWidth',2,'MarkerSize',6);
            currPos2 = self.measured_js(1,1);
            jacob = self.jacob3001(currPos2(1,:)); % This input might be wrong, might need to be joint_values
            joint_vel = currPos2(2,:);
            velKin = jacob .* joint_vel;  
            velocity_x = velKin(1);
            velocity_y = velKin(2);
            velocity_z = velKin(3);
            
            
            %shiver me timbers
            vel_arrow = quiver3(armPos(5,1), armPos(5,2), armPos(5,3), velocity_x, velocity_y, velocity_z, '-','LineWidth',2);
            %quiver3(armPos(5,1), armPos(5,2), armPos(5,3), 100, 100,
            %100);, 
            
                
            title('RBE 3001 Robot Arm')
            xlabel('X Axis (mm)');
            ylabel('Y Axis (mm)');
            zlabel('Z Axis (mm)');
            legend([unga,bunga,chalupa,vel_arrow],'Robot Arm','zAxis of Joint','xAxis of Joint','velocity')
            xlim([-220 220])
            ylim([-220 220])
            zlim([0 300])
            
            hold off
        end
        
%=================================================================================

%=================================================================================      
        function matrix_data = run_trajectoryPlot(self,trajectory_coefficients,motion_time,task_space)
            %data = ["Time" "Theta1 Calculated" "Theta2 Calculated" "Theta3 Calculated" "Actual Theta1" "Actual Theta2" "Actual Theta3"];
            data = []
            curr_pos = self.measured_js(1,0);
            joint_values = curr_pos(1,:);
            tic;
            start = tic;
            [go_die,replace_size] = size(trajectory_coefficients)
            joint1_traj = zeros(1,6);
            joint2_traj = zeros(1,6);
            joint3_traj = zeros(1,6);
            joint1_traj(1,1:replace_size) = trajectory_coefficients(1,:);
            joint2_traj(1,1:replace_size) = trajectory_coefficients(2,:);
            joint3_traj(1,1:replace_size) = trajectory_coefficients(3,:);
            toc;
            t = toc
            next_time = t + 0.2;
            
            while t < motion_time
                t = toc;
                actual_pos = self.measured_js(1,1)
                curr_pos = self.measured_js(1,0);
                pointx = joint1_traj(1) + joint1_traj(2)*t + joint1_traj(3)*t^2 + joint1_traj(4)*t^3 + joint1_traj(5)*t^4+joint1_traj(6)*t^5;
                pointy = joint2_traj(1) + joint2_traj(2)*t + joint2_traj(3)*t^2 + joint2_traj(4)*t^3 + joint2_traj(5)*t^4+joint2_traj(6)*t^5;
                pointz = joint3_traj(1) + joint3_traj(2)*t + joint3_traj(3)*t^2 + joint3_traj(4)*t^3 + joint3_traj(5)*t^4+joint3_traj(6)*t^5;
                angles = self.ik3001([pointx; pointy; pointz]);
                self.servo_jp(transpose(angles));
                dataPoints = self.fdk3001(actual_pos);
                dataPoints = transpose(dataPoints);
                data = [data; t dataPoints];
%                 if mod(motion_time,0.2) == 0 
%                     self.plot_armNODELAY(curr_pos(1,:));
%                     drawnow
%                end
            end
            pause(0.01)
            disp("loop broken")
            matrix_data = data;
        end
        
        function jacobian = jacob3001(self,position)
            theta1 = deg2rad(position(1));
            theta2 = deg2rad(position(2));
            theta3 = deg2rad(position(3));
            
            theta1deg = position(1);
            theta2deg = position(2);
            theta3deg = position(3);
             
            j11 = -100*(sin(theta2) + cos(theta2 + theta3))*sin(theta1);
            j12 =  100*(-sin(theta2+theta3)+cos(theta2))*cos(theta1); %
            j13 = -100*sin(theta2+theta3)*cos(theta1);
            
            j21 = 100*cos(theta1)*cos(theta2+theta3)+100*cos(theta1)*sin(theta2);
            j22 = 100*(-sin(theta2+theta3)+cos(theta2))*sin(theta1);
            j23 = -100*sin(theta1)*sin(theta2+theta3);
            
            j31 = 0;
            j32 =-100*(sin(theta2)+cos(theta2+theta3));
            j33 = -100*cos(theta2+theta3);
            jp = [j11 j12 j13;j21 j22 j23;j31 j32 j33];
            
            jor1 = [0; 0; 1];
            jor2 = [-sind(theta1deg); cosd(theta1deg); 0];
            jor3 = [-sind(theta1deg); cosd(theta1deg); 0];
            jo = [jor1 jor2 jor3];
            
            detJP = det(jp);
            jacobian = [jp; jo];
        end
        
        function velKin = fdk3001(self,measured_jsValues)
            pos = measured_jsValues(1,:);
            vel = measured_jsValues(2,:);
            vel = transpose(vel);
            %WE WERE MISSING THIS vvvvv
            vel = deg2rad(vel);
            
            
            jacob = self.jacob3001(pos);
            velKin = jacob * vel;  
        end
        
        function jpReturn = check_eStop(self)
            
            curr_pos = self.measured_js(1, 0);
            position = curr_pos(1,:);
            theta1 = deg2rad(position(1));
            theta2 = deg2rad(position(2));
            theta3 = deg2rad(position(3));
            
            j11 = -100*(sin(theta2) + cos(theta2 + theta3))*sin(theta1);
            j12 = 100*(-sin(theta2+theta3)+cos(theta2))*cos(theta1);
            j13 = -100*sin(theta2+theta3)*cos(theta1);
            
            j21 = 100*(sin(theta2)+cos(theta2+theta3))*cos(theta1);
            j22 = 100*(-sin(theta2+theta3)+cos(theta2))*sin(theta1);
            j23 = -100*sin(theta1)*sin(theta2+theta3);
            
            j31 = 0;
            j32 =-100*(sin(theta2)+cos(theta2+theta3));
            j33 = -100*cos(theta2+theta3);
            jp = [j11 j12 j13;j21 j22 j23;j31 j32 j33];
            disp(det(jp))
            
            if det(jp) < 100000
                pos = curr_pos(1,:);
                self.servo_jp(pos);
                disp("STOPPED");
                msg = "Singularity has been approached";
                error(msg)
            else
                jpReturn = det(jp);
            end
        end
        
        function deliverOrb(self, color)
%             redDropoff = [-98 96 -80]
%             greenDropoff = [-77 96 -80]
%             orangeDropoff = [98 96 -80]
%             yellowDropoff = [77 96 -80]
            dropOff = [];
            
            if(color == "red")
               dropOff = [-98 96 -80];
            elseif(color == "orange")
               dropOff = [98 96 -80]
            elseif(color == "yellow")
                dropOff = [77 96 -80]
            elseif(color == "green")
                dropOff = [-77 96 -80]
            end
            
            self.interpolate_jp(dropOff,3000);
            pause(4)
            self.openGripper();
            pause(1)
        end
        
        function velMotionPlan(self, targetPointMM)
            %loop runs at 0.001023631672s
            targetPointMM = transpose(targetPointMM);
            currPosDEG = self.measured_js(1,0);
            currPosDEG = currPosDEG(1,:);
            currPosMMfbf = self.fk3001(transpose(currPosDEG));
            currPosMM = currPosMMfbf(1:3,4);
            target_diff = targetPointMM-currPosMM;
            loopCount = 0;
            while(any(abs(target_diff)>5))
                loopCount = loopCount + 1;
                currPosDEG = self.measured_js(1,0);
                currPosDEG = currPosDEG(1,:);
                currPosMMfbf = self.fk3001(transpose(currPosDEG));
                currPosMM = currPosMMfbf(1:3,4);
                target_diff = targetPointMM-currPosMM;
                norman = sqrt((target_diff(1)^2)+(target_diff(2)^2)+(target_diff(3)^2));
                unitVectorMM = target_diff/norman;
                velocity = unitVectorMM * 20;
                jacobian = self.jacob3001(deg2rad(currPosDEG));
                jp = jacobian(1:3,:);
                jointVel = inv(jp) * velocity;
                if(mod(loopCount,200) == 0) 
                    adjVel = rad2deg(jointVel) * 0.2;
                    disp(rad2deg(jointVel))
                    goTo = currPosDEG + transpose(adjVel);
                    goTo(2) = 90 - goTo(2)
                    goTo(3) = 90 - goTo(3)
                    self.servo_jp(goTo);
                end
                     
            end
            self.servo_jp(currPosDEG);
        end
    
        
    end
end

