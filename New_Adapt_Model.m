%% New Adaptation Model

% Mickey Trent
% Neuromechanics Laboratory
% Department of Integrative Physiology
% University of Colorado at Boulder
% Boulder, CO

% Inputs ---- List of trial gains
%        ---- Experimental Data by Subject and Group Average
%        ---- List of Trial gains in cliff condition

% Outputs --- Model prediction for both experimental conditions in Data str

%% Initialize

load('Error_Group2_Mag.mat') % Load Processed Data
load('Error_Group3_Mag.mat')

Error = Error_G3S1; % Change to Subject ID

trial_list = load('gains_into.mat'); % If Gains are away, mult (-1)
trial_list1 = trial_list.gains_into; 
trial_list = load('gains_into_cliff.mat'); % If Gains are away, mult (-1)
trial_list2 = trial_list.gains_into_cliff;


S1 = [1 1 1 1 1 1 1 1 1 1 1]; % Initial guess for Sensitivity

D = 0.0007; % Intitial Guess for the stiffness
A = 0.8; % Initial Guess for the retention factor
erframe = 110; %Obvious

% Gather Data Sets

dat1 = Px(erframe,51:250);
dat2 = Px(erframe,301:500);

%% Optimize with repect to Error Stable Condition

guessEr = [S1, A, D];

Er1 = err_min(trial_list1,dat1,Error.gains,guessEr); % Fit Parameters

D = Er1.D; % New Stiffness ***Can Exlude***
A = Er1.A; % New Retention ***Can Exlude***

Err1 = error_model(Error.gains,Er1.S,D,A,trial_list1); % Run the model using fit S-Vector

ST.A = mod_ad(Err1); % Calculate the adaptation

S.S1 = Er1.S'; % Fit S-Vector

figure; plot((1:1:199),Err1.xn(1:1:199),'-ko'); hold on; plot((1:1:199),Px(erframe,52:250),'-bo')
title('Model Error fit to Experimental Error')
xlabel('Trial Number')
ylabel('Mid-Movement Error (m)')
figure; hold on; plot(Error.gains,ST.A.avadapt_er,'-ks','markerfacecolor','k'); plot(Error.gains,Error.adapt.average.stable,'-bs','markerfacecolor','b')
%% Optimize with Respect to Error Unstable Condition

guessEr2 = [S.S1', A, D];

Er2 = err_min(trial_list2,dat2,Error.gains,guessEr2); % Fit Parameters

D2 = Er2.D;
A2 = Er2.A;

Err2 = error_model(Error.gains,Er1.S,D2,A2,trial_list2);

US.A = mod_ad(Err2);

S.S2 = Er2.S';

figure; plot((1:1:200),Err2.xn(1:1:200),'-ko'); hold on; plot((1:1:200),Px(erframe,301:500),'-ro')
title('Model Error fit to Experimental Error')
xlabel('Trial Number')
ylabel('Mid-Movement Error (m)')
figure; hold on; plot(Error.gains,US.A.avadapt_er,'-ks','markerfacecolor','k'); plot(Error.gains,Error.adapt.average.unstable,'-rs','markerfacecolor','r')



