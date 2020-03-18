function [Error] = error_analysis_both(Px,Py,T,Fx,Fy,lower_bound1,upper_bound1,lower_bound2,upper_bound2,mag)

% Error analysis fucntion to process the robot data for any curl gain set
% Inputs: Px,Py,T
% Outputs: Error information and adaptation


% First define some parameters

N = T.trials;
E = length(Px);
erframe = 110;
erframe1 = 160;
trunk = 275; % Truncate the data after this time frame
cliff = .025; %location of cliff
fail = .09; % failure distance

lower_bound_stable =lower_bound1;
upper_bound_stable =upper_bound1;
lower_bound_unstable = lower_bound2;
upper_bound_unstable = upper_bound2;

% Determine gain set
n = 1;
gains(n) = T.curlgain(1);
for i = 1:N
    for e = 1:size(gains,2)
        if T.curlgain(i) ~= gains(e)
            sig(e) = 1;
        else
            sig(e) = 0;
        end
    end
    if sig ~= zeros(1,n)
        sig = zeros(1,n);
        n = n + 1;
        gains(n) = T.curlgain(i);
    end
end
% Determine magnitude only of gains ****Optional and probably only for
% comparison****

if strcmp('magnitude',mag) == 1
    if gains(5) < 0
        d = 'descend'
        
    else
        d = 'descend'
    end
    gains = abs(gains);
    T.curlgain = abs(T.curlgain);
    gains = sort(gains,2,d);
else
    gains = sort(gains);
end

% Reorder the gain set

%gains = sort(gains);
Error.gains = gains;
g = length(gains);

% Find trials where subjects crossed cliff
n = 0;
for i = 1:N
    if max(Px(:,i)) > cliff
        n = n + 1;
        Error.cliff(n) = i;
    end
    Error.percent_cliff = (n/N)*100;
end

% Determine if any crazy trials exist... ...get rid of them "Failure
% Trials"

num_fail = 0;
for i = 1:N
    if max(Px(:,i)) > fail
        Px(:,i) = NaN;
        num_fail = num_fail + 1;
    end
end

Percent_Fail = num_fail/N


% Truncate the trajectories


for i = 1:N
    for e = trunk:E
        Py(e,i) = NaN;
        Px(e,i) = NaN;
    end
end 

n = 0;
for i = lower_bound_unstable:upper_bound_unstable
    if max(Px(:,i)) > cliff
        n = n + 1;
        Error.cliff_force_trial_number(n) = i;
    end
end

Error.Px = Px;
Error.Py = Py;
Error.T = T;

%% Calculate Stiffness for each condition

Stable_Stiff = NaN(upper_bound1-lower_bound1,1);

for i = lower_bound1:upper_bound1
    Stable_Stiff(i - lower_bound1 + 1) = Fx(erframe1,i)/Px(erframe1,i);
end

Error.Stiff.Stable = Stable_Stiff;

Error.Stiff.average.stable = nanmean(Stable_Stiff,1);

Unstable_Stiff = NaN(upper_bound1-lower_bound1,1);

for i = lower_bound2:(lower_bound2+(upper_bound1-lower_bound1))
    Unstable_Stiff(i - lower_bound2 + 1) = Fx(erframe1,i)/Px(erframe1,i);
end

Error.Stiff.Unstable = Unstable_Stiff;
Error.Stiff.average.unstable = nanmean(Unstable_Stiff,1);
%% Bin the trials by gain

bin = NaN(E,N,size(gains,2));

for i = 1:size(gains,2)
    bin(:,:,i) = binvectors(gains(i),N,T,Px);
end

biny = NaN(E,N,size(gains,2));

for i = 1:size(gains,2)
    biny(:,:,i) = binvectors(gains(i),N,T,Py);
end

% Create phase bins

phasebin_stablex = NaN(E,upper_bound_stable-lower_bound_stable,g);
phasebin_unstablex = NaN(E,upper_bound_unstable-lower_bound_unstable + 1,g);
phasebin_stabley = NaN(E,upper_bound_stable-lower_bound_stable,g);
phasebin_unstabley = NaN(E,upper_bound_unstable-lower_bound_unstable + 1,g);

for i = 1:size(gains,2)
    [phasebin_stablex(:,:,i), phasebin_unstablex(:,:,i)] = phasebin(bin(:,:,i),lower_bound_stable,upper_bound_stable,lower_bound_unstable,upper_bound_unstable);
    [phasebin_stabley(:,:,i), phasebin_unstabley(:,:,i)] = phasebin(biny(:,:,i),lower_bound_stable,upper_bound_stable,lower_bound_unstable,upper_bound_unstable);
end


Error.bin.stable.x = phasebin_stablex;
Error.bin.unstable.x = phasebin_unstablex;
Error.bin.stable.y = phasebin_stabley;
Error.bin.unstable.y = phasebin_unstabley;


%% Calculate the Perp Error at Frame (110) by Bin Non-Normalized (NN)

perp_errorNN_stablex = NaN(E,g);
perp_errorNN_unstablex = NaN(E,g);

for i = 1:g
    perp_errorNN_stablex(:,i) = nanmean(phasebin_stablex(:,:,i),2);
end

avperpNN_stablex = NaN(g,1);
for i = 1:size(gains,2)
    avperpNN_stablex(i) = perp_errorNN_stablex(erframe,i);
end

for i = 1:size(gains,2)
    perp_errorNN_unstablex(:,i) = nanmean(phasebin_unstablex(:,:,i),2);
end

avperpNN_unstablex = NaN(size(gains,2),1);
for i = 1:size(gains,2)
    avperpNN_unstablex(i) = perp_errorNN_unstablex(erframe,i);
end

Error.perpNN.stable = perp_errorNN_stablex;
Error.perpNN.unstable = perp_errorNN_unstablex;
Error.perpNN.avStable = avperpNN_stablex;
Error.perpNN.avUnstable = avperpNN_unstablex;

%% Plot Max Signed Error vs Curl Gain

figure

% Stable

plot(gains,avperpNN_stablex,'-bo','linewidth',2,'markerfacecolor','b','markersize',3); hold on


% Unstable


plot(gains,avperpNN_unstablex,'-ro','linewidth',2,'markerfacecolor','r','markersize',3);


%plot(gains,avperpNN_unstable,'linewidth',2,'color','r'); hold on

xlabel('Curl Gains')
ylabel('Average Perp Error at Frame 110')
grid on
legend('Stable Signed Error','Unstable Signed Error')

%% Find Average Trajectories for each bin in each condition

% traj_sx = NaN(E,1,g);
% traj_sy = NaN(E,1,g);
% traj_usx = NaN(E,1,g);
% traj_usy = NaN(E,1,g);
% 
% for i = lower_bound_stable:upper_bound_stable
%     for e = 1:g
%         if T.curlgain(i) == gains(e)
%             n = size(traj_sx(:,:,e),2) + 1;
%             traj_sx(:,n,e) = Px(:,i);
%             traj_sy(:,n,e) = Py(:,i);
%         end
%     end
% end
% 
% for i = lower_bound_unstable:upper_bound_unstable
%     for e = 1:g
%         if T.curlgain(i) == gains(e)
%             n = size(traj_usx(:,:,e),2) + 1;
%             traj_usx(:,n,e) = Px(:,i);
%             traj_usy(:,n,e) = Py(:,i);
%         end
%     end
% end

avtraj_true_stable_x = NaN(E,g);
avtraj_true_stable_y = NaN(E,g);
avtraj_true_unstable_x = NaN(E,g);
avtraj_true_unstable_y = NaN(E,g);

for i = 1:g
    avtraj_true_stable_x(:,i) = nanmean(phasebin_stablex(:,:,i),2);
    avtraj_true_stable_y(:,i) = nanmean(phasebin_stabley(:,:,i),2);
    avtraj_true_unstable_x(:,i) = nanmean(phasebin_unstablex(:,:,i),2);
    avtraj_true_unstable_y(:,i) = nanmean(phasebin_unstabley(:,:,i),2);
end

Error.avtraj.stablex = avtraj_true_stable_x; Error.avtraj.stabley = avtraj_true_stable_y;
Error.avtraj.unstablex = avtraj_true_unstable_x; Error.avtraj.unstabley = avtraj_true_unstable_y;

figure; plot(avtraj_true_stable_x,avtraj_true_stable_y)
figure; plot(avtraj_true_unstable_x,avtraj_true_unstable_y)
%% Normalize Bins By Average Traj

Error.bin.stable.normx = NaN(size(phasebin_stablex));
Error.bin.unstable.normx = NaN(size(phasebin_unstablex));
Error.number_normalized.stable = zeros(1,g);

for i = 1:size(phasebin_stablex,2)
    for e = 1:g
        if phasebin_stablex(erframe,i,e) > 0 || phasebin_stablex(erframe,i,e) < 0
            Error.bin.stable.normx(:,i,e) = phasebin_stablex(:,i,e) - avtraj_true_stable_x(:,e);
            Error.number_normalized.stable(e) = Error.number_normalized.stable(e) + 1;
        end
    end
end

Error.number_normalized.unstable = zeros(1,g);
for i = 1:size(phasebin_unstablex,2)
    for e = 1:g
        if phasebin_unstablex(erframe,i,e) > 0 || phasebin_unstablex(erframe,i,e) < 0
            Error.bin.unstable.normx(:,i,e) = phasebin_unstablex(:,i,e) - avtraj_true_unstable_x(:,e);
            Error.number_normalized.unstable(e) = Error.number_normalized.unstable(e) + 1;
        end
    end
end

%% Normalize Whole Data Set

Error.normPx = Px;

for i = 1:g
    Error.normPx = normP(gains(i),T,Px,Error.avtraj.stablex(:,i),Error.avtraj.unstablex(:,i),lower_bound_stable,upper_bound_stable,lower_bound_unstable,upper_bound_unstable);
end

%% Calculate the Adaptation

% Create 3D matrices that are the size of the bins, and 11 matrices deep. 

Error.adapt.stable = NaN(E,upper_bound_stable - lower_bound_stable+1,g);
Error.adapt.unstable = NaN(E,upper_bound_unstable - lower_bound_unstable+1,g);

for i = 1:g
    [Error.adapt.stable(:,:,i), Error.adapt.unstable(:,:,i)] = adaptation(gains(i),T,Error.normPx,lower_bound_stable,upper_bound_stable,lower_bound_unstable,upper_bound_unstable);
end

%% Find Average Adaptation
Error.adapt.average.stable = NaN(1,g);
Error.adapt.average.unstable = NaN(1,g);
Error.adapt.average.stable_sd = NaN(1,g);
Error.adapt.average.unstable_sd = NaN(1,g);

for i = 1:g
    Error.adapt.average.stable(i) = nanmean(Error.adapt.stable(erframe,:,i),2);
    Error.adapt.average.unstable(i) = nanmean(Error.adapt.unstable(erframe,:,i),2);
    Error.adapt.average.stable_sd(i) = nanstd(Error.adapt.stable(erframe,:,i));
    Error.adapt.average.unstable_sd(i) = nanstd(Error.adapt.unstable(erframe,:,i));
end

%% Plot Stable Adaptation

% figure; hold on
% errorbar(Error.gains,Error.adapt.average.stable,Error.adapt.average.stable_sd,'-bs','markerfacecolor','b','markersize',2)
% errorbar(Error.gains,Error.adapt.average.unstable,Error.adapt.average.unstable_sd,'-rs','markerfacecolor','r','markersize',2)

figure; hold on
plot(Error.gains,Error.adapt.average.stable,'-bs','markerfacecolor','b','markersize',2);
plot(Error.gains,Error.adapt.average.unstable,'-rs','markerfacecolor','r','markersize',2);

end


