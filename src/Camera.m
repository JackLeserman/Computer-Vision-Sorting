classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;
        cam;
        cam_pose;
        cam_IS;
        size_x;
        size_y;
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(2); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.getCameraPose();
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                camcal; % Change this if you are using a different calibration script
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        
        function [newIs, pose] = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard
            % frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
%             [imagePoints, boardSize] = detectCheckerboardPoints(img);
%             % 4. Compute transformation
%             [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);
%             pose = [   R,    t';
%                     0, 0, 0, 1]
            pose = [0.996047724937052  -0.032406167376463   0.082696855827127 (1.0e+02 * -1.318741161884955);
                    0.088704386159434   0.410357632558937  -0.907600212250252 (1.0e+02 * -0.420437753432049);
                   -0.004523441588193   0.911348700397709   0.411610355505793 (1.0e+02 * 3.456607537704431);
                   0 0 0 1];
        end
        
        %
        function [point, radius] = imageToFilteredHSV(self, image, color)
            self.size_x = size(image,1);
            self.size_y = size(image,2);
            if(color == "red")
                channel1Min = 0.9;
                channel1Max = 0.034;
            elseif(color == "orange")
                channel1Min = 0.00;
                channel1Max = 0.108;
            elseif(color == "yellow")
                channel1Min = 0.108;
                channel1Max = 0.155;
            elseif(color == "green")
                channel1Min = 0.155;
                channel1Max = 0.38;
            end
            saturationMin = 0.30;
            saturationMax = 1.0;
            valueMin = 0;
            valueMax = 1.0;
            
            hsv = rgb2hsv(image);
            
            if(color == "red")
                hsv_filter = ( (hsv(:,:,1) > channel1Min) | (hsv(:,:,1) <= channel1Max) ) & ...
                (hsv(:,:,2) > saturationMin ) & (hsv(:,:,2) <= saturationMax) & ...
                (hsv(:,:,3) > valueMin ) & (hsv(:,:,3) <= valueMax);    
            else
                hsv_filter = ( (hsv(:,:,1) > channel1Min) & (hsv(:,:,1) <= channel1Max) ) & ...
                (hsv(:,:,2) > saturationMin ) & (hsv(:,:,2) <= saturationMax) & ...
                (hsv(:,:,3) > valueMin ) & (hsv(:,:,3) <= valueMax);
            end
            maskedRGBImage = image;
            maskedRGBImage(repmat(~hsv_filter,[1,1,3])) = 0;
            figure
            imshow(maskedRGBImage)
            
            % This is basically just giving us the best fit circle for the
            % color we are looking for
            [centers,radii,strength] = imfindcircles(maskedRGBImage,[5 30]);
            h = viscircles(centers,radii);
            len = length(centers);
            not_done = true;
            counter = 1;
            try
            if(size(centers,1) > 1) % not sure how length or size works but I hope this does it
                while(not_done) % Selects the first considered circle that seems like it would work
                    if(radii(counter) >= 8 && radii(counter) <= 18 && strength(counter) > 0.3) % Making sure ball is correct size
                        center = centers(counter,:);
                        radii = radii(counter);
                        pointX = center(1);
                        pointY = center(2);
                        point = [pointX, pointY];
                        radius = radii(1);
                        not_done = false;
                    end
                    counter = counter + 1; % Will break if it can't find a valid object
                end
            else
                disp("something went wrong here")
                if(radii >= 8 && radii <= 18 && strength > 0.25) % Making sure ball is correct size
                    pointX = centers(1);
                    pointY = centers(2);
                    point = [pointX, pointY];
                    radius = radii(1);
                else
                    disp("No balls were found")
                    point = [-1, -1];
                    radius = 0;
                end
            end
            catch error
                disp("Something went wrong, or there were no balls that were actually in the field")
                disp(error)
                point = [-1, -1];
                radius = 0;
            end
            if radius ~= 0
                text = insertText(maskedRGBImage,point,color);
                figure
                imshow(text)
            end
        end   
        
        
    end
end
