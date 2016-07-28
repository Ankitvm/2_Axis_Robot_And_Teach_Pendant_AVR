%**************************************************************************
% Code:               Code to display Robot Axis Assembly in motion 
% Author:             Ankit Manerikar
% Date:               12/20/2013
%**************************************************************************

clc;
clear all;
close all;

SampleNo = 1;
CheckDataSerial = 0;

RobotSerial = serial('com6');   %Note: Check for the correct
                                %      COM Port from Device Manager

%Plot initialization*******************************************************
figure(1)

legend('Visualization in 2D Workspace');
title('Robotic Assembly Visualization', 'FontSize' , 16 );
PlotArm = plot(zeros(3,1),5*ones(3,1));
axis([(-15),15, (-5), 15 ]);
grid on;
box on;
hold on;
%**************************************************************************

while 1

% Wait while the robot begins transmission    
    fopen(RobotSerial);      
       while ((CheckDataSerial ~= 'T')||(CheckDataSerial ~= 'R') )
        CheckDataSerial = fread(RobotSerial,1);
         if((CheckDataSerial ~= 'T')||(CheckDataSerial ~= 'R'))
             break;
         end
       end
 
% Start Data collection upon receiving start marker       
        SerialData(:,SampleNo) = fread(RobotSerial,3);
       
        %Get Angle Values from the serial port
        GripperAngle = SerialData(1,SampleNo);
        ElbowAngle   = SerialData(2,SampleNo);
        BaseAngle    = SerialData(3,SampleNo);
        
        %Convert values to Radians
        ElbowDispVal = (ElbowAngle + 40)*(pi/180);
        BaseDispVal  = (BaseAngle -50)*(pi/180);
  
        %Get X-Y co-ordinates for plotting
        Joint1_ValX = (-5)*cos(BaseDispVal);
        Joint1_ValY = (5)*sin(BaseDispVal);
    
        Joint2_ValX = (-5)*( cos(BaseDispVal) + cos(BaseDispVal+ ElbowDispVal - pi));
        Joint2_ValY = (5)*( sin(BaseDispVal) + sin(BaseDispVal+ ElbowDispVal - pi));
    
        % Check Mode of operation
        if (CheckDataSerial == 'T')    
            showup = 'TEACH MODE';
        elseif (CheckDataSerial == 'S')
            showup = 'REMEMBER STEP';
        elseif (CheckDataSerial == 'R')
            showup = 'REPEAT MODE';
        end
        
    %Co-ordinates for plotting
    X(SampleNo,:) = [0, Joint1_ValX, Joint2_ValX]; 
    Y(SampleNo,:) = [0, Joint1_ValY, Joint2_ValY]; 
    
    disp(ElbowDispVal); 
    disp(BaseDispVal);

    %Display Robot Position
    delete(PlotArm)
    PlotArm = plot( X(SampleNo,:), Y(SampleNo,:), 'LineWidth', 10, 'Marker', ...
                    'o', 'MarkerSize', 8 ,'MarkerFaceColor', 'Red','MarkerEdgeColor', 'Red' );
    text( -1 , -1, -1,'BASE', 'Fontsize' , 12);
    text((x2-1),(y2-1), 'GRIPPER', 'Fontsize', 12);
    text(0, 12, showup, 'Fontsize', 20);
    hold on;
    
    pause(0.02);     
end