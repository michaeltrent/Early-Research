%VROT_MAIN_BASIC  Processes ROBOT data
%  This script processes ROBOT data from start to finish
%  And calculates basic output variables
%     --called by : n/a
%     --inputs    : filename, datadir
%     --outputs   : data structure(T), +variables
%     --calls     : robotconv.m,diff23f5.m,tangperp_targ.m
%  Last modified  16-Feb-2010

clear all
close all

tic
%% Set directory and File Info
datadir='C:\Documents and Settings\Mickey\My Documents\MATLAB\VROT_Data\VF1S1';
filename='VF1S1';


%% Set the Exponent for the Reward function
rew_exp = str2num(filename(3)); %input('What is the reward exponent? ');
%% Read in Data
[T]=robotdataread_vrot(filename,datadir);

trialdatanames=fieldnames(T);
framedatanames=fieldnames(T.framedata);
configdatanames=fieldnames(T.config);

T.trials = T.config.totaltrials;
T.startposition=repmat([T.config.home_x_cm*0.01 T.config.home_y_cm*0.01 0],T.trials,1);
T.targetposition=[0.01*T.target_x_cm 0.01*T.target_y_cm zeros(T.trials,1)];
maxframe=T.framedata(1).frame(end);

for i= 2:T.trials
    maxframe=max([maxframe T.framedata(i).frame(end)]);
end

%% Find TrialTypes
catchtrials=find(strcmp(T.trialtypename,'clamp')==1);
nulltrials=find(strcmp(T.trialtypename,'null')==1);
forcetrials=find(strcmp(T.trialtypename,'curl_div')==1 | strcmp(T.trialtypename,'curl')==1);
curltrials=find(strcmp(T.trialtypename,'curl')==1);

%% Create Batch Indices
BatchTrials=5;
nbatch=T.trials/BatchTrials;
 
%% Create variable vectors with above trialdata names       
for i=1:length(trialdatanames) eval([trialdatanames{i} '=T.' trialdatanames{i} ';']); end

Px=NaN*zeros(maxframe,T.trials); Py=NaN*zeros(maxframe,T.trials); Pz=zeros(maxframe,T.trials);...
    VPx_m = NaN*zeros(maxframe,T.trials); VPy_m = NaN*zeros(maxframe,T.trials); 
% %% Find if there is a REALLY LONG trial... Typically trial 550
% 
% if length(T.framedata(550).x) > 10000
%     trunk = input('There is a really f-ing long trial in here. Truncate? (y/n)','s');
%     if strcmp(trunk,'y') == 1 
%         trunk_num = input('What frame should we truncate from?');
%     end
%     else
%         trunk_num = length(T.framedata(550).x);
% end

%% Filter Forceplate forces and moments
for i=1:T.trials 
    Frames(i)=max(T.framedata(i).frame); 
    
    Px(1:Frames(i),i)=T.framedata(i).x; Py(1:Frames(i),i)=T.framedata(i).y;
    Vx(1:Frames(i),i)=T.framedata(i).vx; Vy(1:Frames(i),i)=T.framedata(i).vy;
    Fx(1:Frames(i),i)=T.framedata(i).fx; Fy(1:Frames(i),i)=T.framedata(i).fy;
    time(1:Frames(i),i)=T.framedata(i).time; 
    VPx_m(1:Frames(i),i)=T.framedata(i).vrx_m; VPy_m(1:Frames(i),i)=T.framedata(i).vry_m;
end


%% If No batch then batch
 batchnum = 5;
if max(T.batchtrial) < 3
T.batch = ceil(T.trialnumber/batchnum);
end

for i = 1:max(T.batch)
    this_batch = find(T.batch == i);
    T.batchtrial(this_batch) = [1:batchnum]';
end



%% Calculate Resultant Position, Velocity and Motor Force
P=(Px.^2+Py.^2).^0.5;
V=(Vx.^2+Vy.^2).^0.5;
Ff=(Fx.^2+Fy.^2).^0.5;

%% Determine Movement Onset
Plim=0.01*1*T.config.cursor_rad_cm;    %Position Threshold
for i=1:T.trials          
    MovementStartindex(i)=find(P(:,i)>Plim, 1 );
    MovementStart(i)=time(MovementStartindex(i),i);
    t(1:Frames(i),i)=time(1:Frames(i),i)-time(MovementStartindex(i),i);
end

%% Calculate Movement Error
%Calculate tangential and perpendicular error for all frames and trials
%Error will be one vector for each trial
% +ve is CW, -ve is CCW
start=[T.startposition];
stop=[T.targetposition];

tang=NaN*ones(size(Px));
perp=NaN*ones(size(Px));

for trialcounter=1:T.trials
    [tang(1:Frames(trialcounter),trialcounter),perp(1:Frames(trialcounter),trialcounter)]=...
        tangperp_targ([Px(1:Frames(trialcounter),trialcounter) Py(1:Frames(trialcounter),trialcounter) Pz(1:Frames(trialcounter),trialcounter)],start(trialcounter,:),stop(trialcounter,:));
end

%Get Max error for each trial
maxperp=max(abs(perp));

%Remove catch trials
maxperp(catchtrials)=NaN;

%Reshape by batch
maxperp_batch=reshape(maxperp(1:nbatch*BatchTrials),BatchTrials,nbatch);

%Average by batch
meanmaxperp_batch=nanmean(maxperp_batch,1);

%% Calculate catch trial forces
%Calculate tangential and perpendicular force for all frames and trials
%Error will be one vector for each trial
start=[T.startposition];
stop=[T.targetposition];

%get tangential velocity

vtang=NaN(size(Vx));
vperp=NaN(size(Vx));

for trialcounter=1:T.trials
    [vtang(1:Frames(trialcounter),trialcounter),vperp(1:Frames(trialcounter),trialcounter)]=...
        tangperp_targ([Vx(1:Frames(trialcounter),trialcounter) Vy(1:Frames(trialcounter),trialcounter) Pz(1:Frames(trialcounter),trialcounter)],start(trialcounter,:),stop(trialcounter,:));
end

[Vmax,Vmaxi]=max(vtang);

tang=NaN*ones(size(Fx));
perp=NaN*ones(size(Fx));

for trialcounter=1:T.trials
    [tangF(1:Frames(trialcounter),trialcounter),perpF(1:Frames(trialcounter),trialcounter)]=...
        tangperp_targ([Fx(1:Frames(trialcounter),trialcounter) Fy(1:Frames(trialcounter),trialcounter) Pz(1:Frames(trialcounter),trialcounter)],start(trialcounter,:),stop(trialcounter,:));
    maxperpc(trialcounter)=abs(perpF(Vmaxi(trialcounter),trialcounter));
end

%Remove null and force trials
maxperpc(forcetrials)=NaN;
maxperpc(nulltrials)=NaN;

%Reshape by batch
maxperpc_batch=reshape(maxperpc(1:nbatch*BatchTrials),BatchTrials,nbatch);

%Average by batch
meanmaxperpc_batch=nanmean(maxperpc_batch,1);

%% Plot Learning Curves
gaplocation=[10]/5; %specify batch after which you want a gap in the x-axis
%This was 30 edited by MCT 02/24/2010
gapsize=2; %specify size of gap

%[batchaxis1, meanmaxperp_batchplot]=batchplotformat(meanmaxperp_batch,gapsize,gaplocation);



%figure
%plot(batchaxis1,meanmaxperp_batchplot)
%title('Peak Perpendicular Error')
%xlabel('Batch')
%ylabel('Error (m)')

%% Truncate the vectors once they are back at zero.
% for i = 1:size(Px,2)
%     inde2 = find(Vy(:,i) < min(Vy));
% 
% inde = inde1 && inde2;



%% Calcualate the angle

% Center the data about (0,0)

Pxc = Px - abs(T.config.home_x_cm/100); Pyc = Py - T.config.home_y_cm/100;
empt = [];
% Find when the eucildian distance is 3 cm.
euc_dist = sqrt(Pxc.^2 + Pyc.^2);
for i = 1:size(Px,2)
    %dist = find(sqrt(Pxc(:,i).^2 + Pyc(:,i).^2) > .03);
    dist = find(euc_dist(:,i) >0.07);
    out = find(Vy(:,i) >0);
    %er_dx = Px(dist,i); er_dy = Py(dist,i);
    %er_dx = Px(out,i); er_dy = Py(out,i);
    if numel(dist) == 0
        fprintf('Error in Trial %d, no movement',i)
        er_dx = []; er_dy = [];
    else
        er_dx = Pxc(dist(1),i); er_dy = Pyc(dist(1),i);
    end
    %Px_out(1:length(,i) = Px(out,i); Py_out(:,i) = Py(out,i);
    if isempty(er_dx) == 1 || isempty(er_dy) == 1
        fprintf('\nError in trial %d, no euclidian distance less than 4 cm\n',i)
        empt = [empt i];
    else
    error_dist(i,:) = [er_dx(end) er_dy(end)];
    end
end
        
theta = abs(atan(error_dist(:,1)./error_dist(:,2))*180/pi);
if numel(empt) ~= 0
    theta(empt) = NaN;
end
trial_score = 1000*(180 - theta).^rew_exp/180^rew_exp;

theta_vrot = theta;

for i = 1:T.trials
    if i >= T.config.vrottrials(1) && i <= T.config.vrottrials(2)
        theta_vrot(i) = theta_vrot(i) - T.config.vrotangles;
    end
end

trial_score_vrot = 1000*(180 - abs(theta_vrot)).^rew_exp/180^rew_exp;

figure('color','w'); 
subplot(2,1,2); hold on;
plot(1:T.config.vrottrials(1)-1,trial_score_vrot(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,trial_score_vrot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,trial_score_vrot(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Trial Score')

subplot(2,1,1); hold on;
plot(1:T.config.vrottrials(1)-1,theta_vrot(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,theta_vrot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,theta_vrot(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Cursor Angle (Deg)')

%% Calculate the Max euclidian distance

max_dist = max(euc_dist);

% Plot these things

%figure('background' ,'w'); plot(max_dist)
%% Plot the Batch Average

theta_vrot_batch = NaN(1,max(T.batch));
trial_score_batch = NaN(size(theta_vrot_batch));
max_dist_batch = NaN(size(theta_vrot_batch));
theta_batch = NaN(size(theta_vrot_batch));
for i = 1:max(T.batch)
    theta_vrot_batch(i) = nanmean(theta_vrot(find(T.batch == i)));
    theta_batch(i) = nanmean(theta(find(T.batch == i)));
    trial_score_batch(i) = nanmean(trial_score_vrot(find(T.batch == i)));
    max_dist_batch(i) = nanmean(max_dist(find(T.batch == i)));
end
figure('color','w'); 
subplot(2,1,1); hold on
plot(1:T.config.vrottrials(1)/batchnum-1,theta_vrot_batch(1:T.config.vrottrials(1)/batchnum-1),'g','linewidth',2)
plot(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1,theta_vrot_batch(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1),'r','linewidth',2)
plot(T.config.vrottrials(2)/batchnum+2:max(T.batch),theta_vrot_batch(T.config.vrottrials(2)/batchnum+2:end),'b','linewidth',2)
xlabel('Batch Number'); ylabel('Initial Angle (Deg)')

subplot(2,1,2); hold on
plot(1:T.config.vrottrials(1)/batchnum-1,trial_score_batch(1:T.config.vrottrials(1)/batchnum-1),'g','linewidth',2)
plot(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1,trial_score_batch(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1),'r','linewidth',2)
plot(T.config.vrottrials(2)/batchnum+2:max(T.batch),trial_score_batch(T.config.vrottrials(2)/batchnum + 2:end),'b','linewidth',2)
xlabel('Batch Number'); ylabel('Trial Score')

figure('color','w');
subplot(2,1,1); hold on
plot(1:T.config.vrottrials(1)-1,max_dist(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,max_dist(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,max_dist(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Max Euclidian Distance (cm)')
subplot(2,1,2); hold on
plot(1:T.config.vrottrials(1)/batchnum-1,max_dist_batch(1:T.config.vrottrials(1)/batchnum-1),'g','linewidth',2)
plot(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1,max_dist_batch(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1),'r','linewidth',2)
plot(T.config.vrottrials(2)/batchnum+2:max(T.batch),max_dist_batch(T.config.vrottrials(2)/batchnum + 2:end),'b','linewidth',2)
xlabel('Batch Number'); ylabel('Batch Average Max Euclidian Distance')
 %% Fit learning curves 
% guess = [10,10];
% fh = @(dat,trial,p) dat(51)+(dat(51) - p(1))*exp(-trial(52:500)/p(2));
% err = @(theta_vrot,trial,p) (sum(theta_vrot(52:500) - fh(theta_vrot,trial,p)).^2);
% 
% fits = fminsearch(@(p) err(-theta_vrot,T.trialnumber,p),guess);

 %% Call fit data
figure('color','w')
p = FitExponential_aa(theta_vrot_batch(10:100),(10:100));

gro = str2num(filename(2));
SubID = str2num(filename(end));

save_to_text_file(p',SubID,'p13');

%% Find when sub returns to baseline



%% Find hand paths out... Back will be later

endpoint = NaN(size(Px,2),1);

 for i = 1:size(Px,2)
     clear temp
     temp = find(euc_dist(:,i) == max_dist(i),1,'first');
     if numel(temp) ~= 1
         endpoint(i) = temp(1);
     elseif numel(temp) == 0 
         endpoint(i) = NaN;
     else
         endpoint(i) = temp;
     end
 end

% Start by finding the max and min velocities

% for i = 1:size(Px,2)
%     out1(i) = find(Vy(:,i) == max(Vy(:,i)));
%     back(i) = find(Vy(:,i) == min(Vy(:,i)));
%     Endpoint(i) = find(V(out1(i):back(i),i) == min(V(out1(i):back(i),i)));
% end
max_V = max(V);

figure('color','w')
plot(Px(1:endpoint,1),Py(1:endpoint,1))

%% Plot handpaths out

outx = NaN(size(Px));
outy = NaN(size(Py));
backx = NaN(size(Px));
backy = NaN(size(Px));

for i = 1:size(Px,2)
    outx(1:endpoint(i),i) = Pxc(1:endpoint(i),i);
    outy(1:endpoint(i),i) = Pyc(1:endpoint(i),i);
    backx(endpoint:end,i) = Pxc(endpoint:end,i);
    backy(endpoint:end,i) = Pyc(endpoint:end,i);
end

figure('color','w'); 
subplot(1,2,1)
plot(outx,outy)
title('Handpaths Out')
xlabel('x-position (m)')
ylabel('y-position (m)')
axis equal
subplot(1,2,2)
plot(backx,backy)
title('Handpaths Back')
xlabel('x-position (m)')
ylabel('y-position (m)')
axis equal

%% Find Endpoint Angle

theta_end = NaN(size(theta_vrot));
theta_end_rot = NaN(size(theta_vrot));

for i = 1:size(Px,2)
    theta_end(i) = atand((Pxc(endpoint(i),i)/Pyc(endpoint(i),i)));
    if i >= T.config.vrottrials(1) && i <= T.config.vrottrials(2)
        theta_end_rot(i) = theta_end(i) - T.config.vrotangles;
    else
        theta_end_rot(i) = theta_end(i);
    end
end

% Ratio of intial to endpoint

i_e_rat = theta_vrot./theta_end_rot;
mm = nanmean(i_e_rat(1:50));
stan = nanstd(i_e_rat(1:50));
for i = 1:size(Px,2)
    if  i_e_rat(i) < mm - 3*stan || i_e_rat(i) > mm + 3*stan
        max_p = max(Pxc(:,i));
        pos = find(Pxc(:,i) == max_p);
        Pxc(pos,i) =NaN; Pyc(pos,i) = NaN;
        i_e_rat(i) = 0;
    end
end
    
figure('color','w')
subplot(2,1,1); hold on;
plot(1:T.config.vrottrials(1)-1,theta_end(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,theta_end(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,theta_end(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Cursor Endpoint Angle (Deg)')

subplot(2,1,2); hold on;
plot(1:T.config.vrottrials(1)-1,theta_end_rot(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,theta_end_rot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,theta_end_rot(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Cursor Endpoint Angle (Deg) Corrected')

%% Plot Both Endpoint and Initial Angle

figure('color','w'); title('Initial (Top) and Endpoint (bottom) Angle')

subplot(2,1,1); hold on;
plot(1:T.config.vrottrials(1)-1,theta_vrot(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,theta_vrot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,theta_vrot(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Cursor Angle (Deg)')

subplot(2,1,2); hold on;
plot(1:T.config.vrottrials(1)-1,theta_end_rot(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,theta_end_rot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,theta_end_rot(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Cursor Endpoint Angle (Deg)')

figure('color','w'); hold on
subplot(2,1,1); hold on
plot(1:T.config.vrottrials(1)-1,i_e_rat(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,i_e_rat(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,i_e_rat(T.config.vrottrials(2)+1:end),'g','linewidth',2)
xlabel('Trial Number'); ylabel('Initial:Endpoint Angles Ratio')
title('Initial to Endpoint angle ratio')
subplot(2,1,2); hold on
plot(1:T.config.vrottrials(1)-1,theta_vrot(1:T.config.vrottrials(1)-1),'bo','markersize',6)
plot(1:T.config.vrottrials(1)-1,theta_end_rot(1:T.config.vrottrials(1)-1),'b^','markersize',6)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,theta_vrot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'ro','markersize',6)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,theta_end_rot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r^','markersize',6)
plot(T.config.vrottrials(2)+1:T.trials,theta_vrot(T.config.vrottrials(2)+1:end),'go','markersize',6)
plot(T.config.vrottrials(2)+1:T.trials,theta_end_rot(T.config.vrottrials(2)+1:end),'g^','markersize',6)
xlabel('Trial Number'); ylabel('Angle (Deg)')
title('Initial and Endpoint angles')

figure; hold on
plot(1:T.config.vrottrials(1)-1,max_V(1:T.config.vrottrials(1)-1),'b','linewidth',2)
plot(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1,max_V(T.config.vrottrials(1)+1:T.config.vrottrials(2)-1),'r','linewidth',2)
plot(T.config.vrottrials(2)+1:T.trials,max_V(T.config.vrottrials(2)+1:end),'g','linewidth',2)

%% Batch the end_point angles

theta_end_batch = NaN(1,max(T.batch));
max_V_batch = NaN(size(theta_vrot_batch));
i_e_rat_batch = NaN(size(theta_vrot_batch));
for i = 1:max(T.batch)
    theta_end_batch(i) = nanmean(theta_end_rot(find(T.batch == i)));
    max_V_batch(i) = nanmean(max_V(find(T.batch == i)));
    i_e_rat_batch(i) = nanmean(i_e_rat(find(T.batch == i)));
end

figure('color','w');
subplot(2,1,2); hold on
plot(1:T.config.vrottrials(1)/batchnum-1,theta_end_batch(1:T.config.vrottrials(1)/batchnum-1),'g^','markersize',4)
plot(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1,theta_end_batch(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1),'r^','markersize',4)
plot(T.config.vrottrials(2)/batchnum+2:max(T.batch),theta_end_batch(T.config.vrottrials(2)/batchnum + 2:end),'b^','markersize',4)
plot(1:T.config.vrottrials(1)/batchnum-1,theta_vrot_batch(1:T.config.vrottrials(1)/batchnum-1),'go','markersize',4)
plot(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1,theta_vrot_batch(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1),'ro','markersize',4)
plot(T.config.vrottrials(2)/batchnum+2:max(T.batch),theta_vrot_batch(T.config.vrottrials(2)/batchnum + 2:end),'bo','markersize',4)
xlabel('Batch Number'); ylabel('Angle (deg)')
subplot(2,1,1); hold on
plot(1:T.config.vrottrials(1)/batchnum-1,i_e_rat_batch(1:T.config.vrottrials(1)/batchnum-1),'g','linewidth',2)
plot(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1,i_e_rat_batch(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1),'r','linewidth',2)
plot(T.config.vrottrials(2)/batchnum+2:max(T.batch),i_e_rat_batch(T.config.vrottrials(2)/batchnum + 2:end),'b','linewidth',2)
xlabel('Batch Number'); ylabel('Initial:Endpoint angle ratio')

figure('color','w'); hold on
plot(1:T.config.vrottrials(1)/batchnum-1,max_V_batch(1:T.config.vrottrials(1)/batchnum-1),'g','linewidth',2)
plot(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1,max_V_batch(T.config.vrottrials(1)/batchnum+1:T.config.vrottrials(2)/batchnum-1),'r','linewidth',2)
plot(T.config.vrottrials(2)/batchnum+2:max(T.batch),max_V_batch(T.config.vrottrials(2)/batchnum + 2:end),'b','linewidth',2)


%% Calculate the "Learning Rate"

% Define the trial to examine

learn_trial = T.config.vrottrials(1) + 70;
% Determine average perfornmance in null

base = nanmean(theta_vrot(1:T.config.vrottrials(1) - 1));
base_std = nanstd(theta_vrot(1:T.config.vrottrials(1) - 1));

theta_learn = nanmean(theta_vrot(learn_trial:learn_trial+20))-base 


%% Find Beginning of exploration

% Define the length of consecutive trials to search through

num_tri = 3; % Relatively arbitrarty

% Use existing values for the baseline performance as well as std

poss_ex = find(abs(theta(T.config.vrottrials(1):T.config.vrottrials(2))) > (base + base_std)) + T.config.vrottrials(1)-1;
ex = [];
for i = 1:length(poss_ex) - num_tri
    if poss_ex(i+num_tri) == poss_ex(i)+num_tri
        ex = [ex poss_ex(i)]; % This should be the periods of exploration
    end
end

%%  %% Call fit data again after subject begins exploring
figure('color','w')
if ex(1) > 70
    setback = 5;
else 
    setback = 1;
end
p2 = FitExponential_aa(theta_vrot_batch(ceil(ex(1)/5)-setback:100),(ceil(ex(1)/5)-setback:100));

% Save the 'p' values

save_to_text_file(p2',SubID,'p23');


%% Calculate the mean angle and std at late learning


late(1) = nanmean(theta_vrot(450:500));
late(2) = nanstd(theta_vrot(450:500));

save_to_text_file(late',SubID,'late3')

%% Remove outlier points
% ind2 = find(theta > 40);
% ind = find(theta_batch > 40);
% 
% theta_batch(ind) = 30;
% theta(ind2) = 30;

prob_nan = find(isnan(theta) == 1);
if numel(prob_nan) ~= 0
    theta(prob_nan) = nanmean(theta(prob_nan - 2):theta(prob_nan+2));
end
%% Generalized regression fit
% First define the intial  guess

x0 = [p2(3)/10 1 150];
param = [nanmean(theta(1:50)) nanmean(theta(450:500)) ex(1)];
y = gen_logit_fit2(x0,param,[1:500],theta(1:500))
%% Generalized logistic fit BATCH
x0 = [p2(3) 20 10];
param = [nanmean(theta_batch(1:9)) nanmean(theta_batch(95:100)) ceil(ex(1)/5)];

yb = gen_logit_fit2(x0,param,[1:100],theta_batch(1:100))

%% Gen fit just the slope

x0 = [y(4)];
param = [y(1:3) y(5:6)];

y_gen1 = gen_logit_fit1(x0,param,1:500,theta(1:500))

%% again for batch

x0 = [yb(4)];
param = [yb(1:3) yb(5:6)];

y_gen1b = gen_logit_fit1(x0,param,1:100,theta_batch(1:100))

%% Save handpaths out

%  filenamex = ['HandPaths_Outx' num2str(SubID)];
% save_to_text(outx,[],filenamex);
% 
%  filenamey = ['HandPaths_Outy' num2str(SubID)];
% save_to_text(outy,[],filenamey);

% save(filenamex,'outx')
% save(filenamey,'outy')
filenamex = ['Init_Ang' num2str(SubID)];

save(filenamex,'theta_vrot')

%% Save Trial score and STD

av_score = nanmean(trial_score_vrot(450:500));
std_score = nanstd(trial_score_vrot(450:500));

filenam = ['av_score_group' num2str(gro) 'Sub' num2str(SubID)];
filenamstd = ['std_score_group' num2str(gro) 'Sub' num2str(SubID)];
save(filenam, 'av_score')
save(filenamstd,'std_score')
%% Close out the timer
toc