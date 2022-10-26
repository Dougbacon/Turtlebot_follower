function Group7_Project2()
%% Run this section first to establish ROS connection to Turtlebot
clear all
close all
rosshutdown
robot= Turtlebot_main2() % Create a Turtlebot object with all the ROS topic subscribed and published

%% Main Code

while(1)
    velMsg=zeros(1,6); % Create empty velocity matrix
    disp('Looking for QR code')
    [detectQR, leadPose]= robot.featureDetection(); %Intiate the feature detection
    
    while detectQR == 0 %Return 0 if QR code not found
        disp('No QR code detected')
        tic
        velMsg(1,6) = 0.2; % Set angular z velocity (turn anti-clockwise)
        while(1)
            robot.setCmdMsg(velMsg); % Publish the velocity matrix to Turtlebot
            if toc >= (0.5)
                velMsg(1,6) = 0; 
                robot.setCmdMsg(velMsg); % Keep rotating on the same spot until QR is detected
                break;
            end
        end
        [detectQR, leadPose]=robot.featureDetection();
        if detectQR==1 % Return 1 if the QR is detected
            break;
        end
    end
    if detectQR(1)==1
        disp('QR code detected')


        
%         IntersectPose = robot.generateIntersectPoint(leadPose)
        lidarDist=robot.getLidarMsg(); % Simulataneouly get the Lidar scan readings
        velMsg=zeros(1,6);

        
        


        if lidarDist(1,1)> 0.5 % Check if any object in front is further than 0.7m away
            

            disp('Following QR code')
            [detectQR, leadPose]=robot.featureDetection();
            robot.rotateToQR(leadPose); % Rotate the Turtlebot to the Guider pose
            
            IntersectPose = robot.generateIntersectPoint(leadPose);

            if robot.checkPosition(IntersectPose) == 1
            disp("At Intersection point")
            velMsg(1,1) = 0;     % meters per second
            robot.setCmdMsg(velMsg);

            % If at Intersect point rotate to face QR
            [detectQR, newleadPose]= robot.featureDetection();
            
            robot.rotateToQR(newleadPose);  
            
            else
            disp("Distance to Guider greater than 0.5")
            velMsg(1,1) = 0.1;     % Set linear velocity x to 0.1 m/s
            robot.setCmdMsg(velMsg);
            end
        elseif lidarDist(1,1) <= 0.5 % Stop Turtlebot if object in front is 0.7m or less away
            robot.rotateToQR(leadPose);
            disp("Distance to Guider less than 0.5")
            velMsg(1,1) = 0;     % meters per second
            robot.setCmdMsg(velMsg);
            
        end
    end

end

%% Test Turtlebot Class Functions
detectQR=robot.featureDetection(1)
followerOdo= robot.getOdometry()
guiderOdo=robot.getGuiderOdometry()
lidarDist=robot.getLidarMsg()
cmdMsg=robot.getCmdMsg()
[angle,direction]=robot.checkPositionToQR()
%%
rostopic list
%%
clear
rosshutdown
%%
end