classdef Turtlebot_main
    properties
        % Set the Turtlebot class variables
        lidarSub;
        odom1Sub;
        odom2Sub
        readMarker;
        rgbSub;
        followerCmd;
        depthSub;

        ipaddress= "http://localhost:11311"; % IP address of the host

     


    end
    methods
        
        function self = Turtlebot_main()
            rosinit(self.ipaddress) % Create ROS bridge to Turtlebot
            self.readMarker = rgb2gray(imread('marker1.jpg')); % Read QR code
            self.rgbSub = rossubscriber('/follower/camera/rgb/image_raw'); % Subscribe to RGB camera
            self.depthSub = rossubscriber('/follower/camera/depth/image_raw',"DataFormat","struct");
            self.lidarSub = rossubscriber('/follower/scan',"DataFormat","struct"); % Subscribe to lidar scan
            self.odom1Sub = rossubscriber('/follower/odom',"DataFormat","struct"); % Subscribe to Odometry to follower
            self.odom2Sub = rossubscriber('/leader/odom',"DataFormat","struct"); % Subscribe to Odometry to guider
            self.followerCmd = rospublisher('/follower/cmd_vel',"DataFormat","struct"); % Publish move commands to follower



        end

        function detected = featureDetection(self)
            rgbMsg = receive(self.rgbSub,10); %Recieve RGB camera messaes
            [rgbImage,~]=readImage(rgbMsg); % Read the image
            original= imrotate(self.readMarker,-90,'bilinear','crop'); % Rotate the marker so it is the same orientation as the RGB image


            rgb2GrayCam=rgb2gray(rgbImage); % Converted to grayscale
            cam = imlocalbrighten(rgb2GrayCam); % Adjust the image but increase brightness

            marker1=detectSURFFeatures(original);% Detect using SURF feature
            marker2=detectSURFFeatures(cam);
            %extract the point and descriptor from image
            [features1, validPoints1]= extractFeatures(original,marker1);
            [features2, validPoints2]= extractFeatures(cam,marker2);

            indexPairs=matchFeatures(features1,features2);
            matchedPoints1=validPoints1(indexPairs(:,1));
            matchedPoints2=validPoints2(indexPairs(:,2));
            if size(indexPairs,1)<10 % Check if there is enough SURF points for feature detection
                detected=0;
            else
                [tform,inlierDistorted,inlierOriginal]=estimateGeometricTransform(matchedPoints2, matchedPoints1,'similarity'); %Remove the outlier points

                %             showMatchedFeatures(self.readMarker,cam,inlierOriginal,inlierDistorted,'montage');
                count= inlierOriginal.Count; % Get the size of all valid inlier points
                if count >= 5 % If there are more than 5, the QR code detected is valid
                    showMatchedFeatures(self.readMarker,cam,inlierOriginal,inlierDistorted,'montage');
                    detected= 1;
                    disp("QR detected")

                elseif count < 5 % Less than 5 points not valid
                    detected= 0;
                    disp("QR not detected")

                end
            end

           
        end

        function currentOdo = getOdometry(self)
            odomMsg = receive(self.odom1Sub,10); %Recieve odomometry messages
            currentOdo=zeros(1,4); % Creare odometry matrix (x y z theta)
            pose = odomMsg.Pose.Pose;
            currentOdo(1,1) = pose.Position.X; % Position X
            currentOdo(1,2) = pose.Position.Y; % Position Y
            currentOdo(1,3) = pose.Position.Z; % Poistion Z
            quat = pose.Orientation;

            angles = quat2eul([quat.W quat.X quat.Y quat.Z]) 
            currentOdo(1,4) = rad2deg(angles(1)) % Convert to degrees
        end

        function currentOdo = getGuiderOdometry(self)
            odomMsg = receive(self.odom2Sub,10); %Recieve odomometry messages
            pose = odomMsg.Pose.Pose;
            x = pose.Position.X; % Position X
            y = pose.Position.Y; % Position Y
            z = pose.Position.Z; % Poistion Z
            quat = pose.Orientation;
            robotOdo=[x y z]; % Current XYZ position
            robotTransl = trvec2tform(robotOdo); % Transform translation into homogeneous matrix
            robotQuat = quaternion([quat.W quat.X quat.Y quat.Z]); %Convert the orientation into quaternion form
            rotmXYZ = rotmat(robotQuat,'point'); %Convert quarternion into rotation matrix
            robotRot = rotm2tform(rotmXYZ); % Transform rotation into homogeneous matrix
            currentOdo= robotTransl*robotRot; % Homogeneous matrix for both rotation and translation

            angles = quat2eul([quat.W quat.X quat.Y quat.Z])
            theta = rad2deg(angles(1)) % Convert to degrees
        end

        function newLidarDist= getLidarMsg(self)
            scanMsg=receive(self.lidarSub); %Recieve lidar messages
            lidarDist= scanMsg.Ranges; %Get all the range values
            newLidarDist= lidarDist((isfinite(lidarDist))); % Filter out all the infinite values 
        end
        function newCmdMsg= getCmdMsg(self)
                cmdMsg = rosmessage(self.followerCmd); % Get the velocity messages
                newCmdMsg=[cmdMsg.Linear.X cmdMsg.Linear.Y cmdMsg.Linear.Z cmdMsg.Angular.X cmdMsg.Angular.Y cmdMsg.Angular.Z]; % Put into a matrix  

        end
        function setCmdMsg(self, velMsg)
            cmdMsg = rosmessage(self.followerCmd); 
            cmdMsg.Linear.X= velMsg(1,1);
            cmdMsg.Linear.Y= velMsg(1,2);
            cmdMsg.Linear.Z= velMsg(1,3);
            cmdMsg.Angular.X= velMsg(1,4);
            cmdMsg.Angular.Y=velMsg(1,5);
            cmdMsg.Angular.Z=velMsg(1,6);
            send(self.followerCmd,cmdMsg); % Convert function input into velocity message and publish it

        end
        function [angleDiff, direction]= checkPositionToQR(self)
            odom1Msg = receive(self.odom1Sub,10); %Recieve odomometry messages
            pose1 = odom1Msg.Pose.Pose;
            x1 = pose1.Position.X; % Position X
            y1 = pose1.Position.Y; % Position Y
            z1 = pose1.Position.Z; % Poistion Z
            followerQuat = pose1.Orientation;
            angles1 = quat2eul([followerQuat.W followerQuat.X followerQuat.Y followerQuat.Z])
            theta1 = rad2deg(angles1(1)) %Pose angle for follower

            odom2Msg = receive(self.odom2Sub,10); %Recieve odomometry messages
            pose2 = odom2Msg.Pose.Pose;
            x2 = pose2.Position.X; % Position X
            y2 = pose2.Position.Y; % Position Y
            z2 = pose2.Position.Z; % Poistion Z
            guiderQuat = pose2.Orientation;
            angles2 = quat2eul([guiderQuat.W guiderQuat.X guiderQuat.Y guiderQuat.Z])
            theta2 = rad2deg(angles2(1)) %Pose angle for guider

            angleDiff= theta2-theta1; % Get the difference between each robot
            if angleDiff < 0
                disp('Rotate clockwise')
                direction=0;
            elseif angleDiff > 0
                disp('Rotate anti-clockwise')
                direction=1;
            end

        end
        function rotateToQR(self)

            [angle,direction]=self.checkPositionToQR(); %Get the angle between robots and the rotating direction
            if abs(angle) >= 0.5 && direction ==1 %Check if angle greater than 0.5 turn anti-clockwise
                aligned=0; %Check if Turtlebot is aligned with guider
                velMsg=zeros(1,6); %Empty velocity matrix
                followerOdo=self.getOdometry();% Get follower odometry
                [angle,direction]=self.checkPositionToQR();
                totalAngle= angle+followerOdo(1,4);
                while aligned==0 %Not aligned
                    disp('Anti-clockwise')
                    followerOdo=self.getOdometry();
                    velMsg(1,6)=0.1;
                    self.setCmdMsg(velMsg) % Rotate anti-clockwise to guider
                    if followerOdo(1,4) >= totalAngle 
                        velMsg(1,6)=0;
                        self.setCmdMsg(velMsg);
                        disp('Aligned')
                        aligned=1; %Stop when turtlebot is facing the guider
                    end


                end
            elseif abs(angle) >=0.5 && direction ==0 %Check if angle greater than 0.5 turn clockwise
                disp('Clockwise')
                aligned=0;
                velMsg=zeros(1,6);
                followerOdo=self.getOdometry();
                [angle,direction]=self.checkPositionToQR();
                totalAngle= angle+followerOdo(1,4)
                while aligned==0
                    followerOdo=self.getOdometry();
                    velMsg(1,6)=-0.1;
                    self.setCmdMsg(velMsg) % Publish negative velocity to turn clockwise
                    if followerOdo(1,4) <= totalAngle
                        velMsg(1,6)=0;
                        self.setCmdMsg(velMsg);
                        disp('Aligned') % Stop when turtlebot is facing the guider
                        aligned=1;
                    end


                end
            end
        end


    end 



end