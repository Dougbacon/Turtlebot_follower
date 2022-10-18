function Group7_Project()
%% Run this section first to establish ROS connection to Turtlebot
clear all
close all

robot= Turtlebot_main() % Create a Turtlebot object with all the ROS topic subscribed and published

%% Main Code

while(1)
    velMsg=zeros(1,6); % Create empty velocity matrix
    disp('Looking for QR code')
    detectQR= robot.featureDetection(); %Intiate the feacture detection
    
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
        detectQR=robot.featureDetection();
        if detectQR==1 % Return 1 if the QR is detected
            break;
        end
    end
    if detectQR==1
        disp('QR code detected')
        lidarDist=robot.getLidarMsg(); % Simulataneouly get the Lidar scan readings
        velMsg=zeros(1,6);
        if lidarDist(1,1)> 0.7 % Check if any object in front is further than 0.7m away
            disp('Following QR code')
            robot.rotateToQR(); % Rotate the Turtlebot to the Guider pose
            disp("Distance to Guider greater than 0.5")
            velMsg(1,1) = 0.1;     % Set linear velocity x to 0.1 m/s
            robot.setCmdMsg(velMsg);
            
        elseif lidarDist(1,1) <= 0.7 % Stop Turtlebot if object in front is 0.7m or less away
            robot.rotateToQR();
            disp("Distance to Guider less than 0.5")
            velMsg(1,1) = 0;     % meters per second
            robot.setCmdMsg(velMsg);
            
        end
    end

end

%% Test Turtlebot Class Functions
detectQR=robot.featureDetection()
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