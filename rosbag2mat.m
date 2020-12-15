% Extract data from recording of welding robot traveling in or around the ship's double hull block
% 16833 SLAM course project
% Group 1
% last modified 12/15/2020
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% This script pre-processes recorded data from a welding robot motion
%%%% trajectory and save them as .mat file for later processing.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% bag1 = rosbag('slam_data1.bag');
% bag1 = rosbag('slam_data2.bag');
bag1 = rosbag('slam_data3.bag');
% bag1 = rosbag('inblock_motion_select.bag');
% bag1 = rosbag('inblock_with_Z.bag'); %dataset with height adjustment


%%
Sel1 = select(bag1, 'Topic', '/gazebo/relative_dist');
Sel2 = select(bag1, 'Topic', '/longi_distance/longi_dist');
Sel3 = select(bag1, 'Topic', '/longi_distance/longi_angle');
Sel4 = select(bag1, 'Topic', '/tf');

msgStructs1 = readMessages(Sel1, 'DataFormat','struct');
msgStructs2 = readMessages(Sel2, 'DataFormat','struct');
msgStructs3 = readMessages(Sel3, 'DataFormat','struct');
msgStructs4 = readMessages(Sel4, 'DataFormat','struct');

%% Go thru time to add time stamp THIS SECTION MAY TAKE 2~3min depending on the length of bag
timeGT = -ones(size(msgStructs1));
timeRSdist = -ones(size(msgStructs2));
timeRSangle = -ones(size(msgStructs3));
timeLD = -ones(size(msgStructs4));

idx_gt = 0; idx_rsd = 0; idx_rsa = 0; idx_ld = 0; %counter for the each measurements
tic
for i=1:bag1.NumMessages
    type = bag1.MessageList{i,2};
    timestamp = bag1.MessageList{i,1};
    if type==categorical({'/gazebo/relative_dist'})
        idx_gt = idx_gt+1;
        timeGT(idx_gt) = timestamp;
    elseif type==categorical({'/longi_distance/longi_dist'})
        idx_rsd = idx_rsd+1;
        timeRSdist(idx_rsd) = timestamp;
    elseif type==categorical({'/longi_distance/longi_angle'})
        idx_rsa = idx_rsa+1;
        timeRSangle(idx_rsa) = timestamp;
    elseif type==categorical({'/tf'})
        idx_ld = idx_ld+1;
        timeLD(idx_ld) = timestamp;
    end
end
toc  %took ~180 seconds to label the timestamps for 210k measurements

timeGT = timeGT - bag1.StartTime;
timeRSdist = timeRSdist - bag1.StartTime;
timeRSangle = timeRSangle - bag1.StartTime;
timeLD = timeLD - bag1.StartTime;

%% Extract Ground Truth longi measurement
gTlength = size(msgStructs1,1);

gTLongiLeft = zeros(gTlength,1);
gTLongiCenter = zeros(gTlength,1);
gTLongiRight = zeros(gTlength,1);
gTLongiAngle = zeros(gTlength,1);
gTX = zeros(gTlength,1); 
gTY = zeros(gTlength,1);

for i=1:gTlength
    gTLongiLeft(i) = msgStructs1{i}.LongiLeft;
    gTLongiCenter(i) = msgStructs1{i}.LongisCenter;
    gTLongiRight(i) = msgStructs1{i}.LongiRight;
    gTLongiAngle(i) = msgStructs1{i}.Angle;
    
    gTX(i) = msgStructs1{i}.BaseX;
    gTY(i) = msgStructs1{i}.BaseY;
end

%% Extract measurement on local landmarks/features
RsLength = size(msgStructs2,1);
RsY = zeros(RsLength,1);
Rslongi_angle = zeros(RsLength,1);

for j=1:RsLength
    Rslongi_angle(j) = msgStructs3{j}.Data;
    
%     Rslongi_dist
    if abs(Rslongi_angle(j))<90.0 %traveling to left, measuring right longi
        RsY(j) = msgStructs2{j}.Data - 0.25; 
    else
        RsY(j) = 0.5 - msgStructs2{j}.Data; % measuring left longi, 
    end
end

%% Obtain result of HDL localization from /tf recording
LdLength = size(msgStructs4,1);
Ld_Tran = cell(LdLength,1);
Ld_Quat = cell(LdLength,1);
for k=1:LdLength
%     disp(msgStructs4{k,1}.Transforms.Transform.Translation.X);
    if size(msgStructs4{k,1}.Transforms,2)>1 
        if k==1
            Ld_Tran(k) = Ld_Tran(k+1);  %if first one, then take on the value of the second
            Ld_Quat(k) = Ld_Quat(k+1);
        else
            Ld_Tran(k) = Ld_Tran(k-1);  %zero order hold from previous value
            Ld_Quat(k) = Ld_Quat(k-1);
        end
    else
        Ld_Tran(k) = {msgStructs4{k,1}.Transforms.Transform.Translation};
        Ld_Quat(k) = {msgStructs4{k,1}.Transforms.Transform.Rotation};
    end
end
% LdXdist(LdXdist==0) = nan;

%% Plot ground truth trajectory
totaltime = bag1.EndTime-bag1.StartTime;

filename = 'bag3_stage1.mat';
save(filename,'totaltime','Ld_Tran','Ld_Quat','LdLength','gTX','gTY','gTLongiAngle','gTlength','RsLength','RsY','Rslongi_angle','timeGT','timeLD','timeRSdist','timeRSangle');

%%
figure(1) 
hold on
plot(timeGT,gTY,'g','LineWidth',3)
plot(timeRSdist,RsY,'r','LineWidth',3)
legend('GroundTruth Y Distance', 'RealSense Y Distance with time stamping','FontSize',16);
title('Longi-distance with appropriate timestamping','FontSize',20);
xlabel('time (sec)','FontSize',16); ylabel('longi distance (meter)','FontSize',16);hold off

