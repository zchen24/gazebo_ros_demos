% Zihan Chen 
% 2018-06-01 
% Verify Potential Energy

clc; clear; close all;

% read data from bag file 
% bag = ros.Bag('2018-06-01-12-13-47.bag');
% bag = ros.Bag('2018-06-01-15-02-14.bag');
% bag = ros.Bag('2018-06-01-19-14-33.bag');
% bag = ros.Bag('2018-06-01-19-23-00.bag');
bag = ros.Bag('2018-06-01-19-51-01.bag');

jnts = bag.readAll('/rrbot/joint_states');
% jnts = jnts(1501:3500);
jnts_array = [jnts{:}];

apos = [jnts_array.position];
avel = [jnts_array.velocity];
atrq = [jnts_array.effort];

atime = zeros(1, length(jnts));
for i = 1:length(jnts)
    atime(i) = jnts{i}.header.stamp.time;
end

figure; 
subplot(3,1,1); plot(atime, apos(1,:), 'r', atime, apos(2,:), 'b'); 
grid on; legend('jnt 1', 'jnt 2');

subplot(3,1,2); plot(atime, avel(1,:), 'r', atime, avel(2,:), 'b'); 
grid on; legend('vel 1', 'vel 2');

subplot(3,1,3); plot(atime, atrq(1,:), 'r', atime, atrq(2,:), 'b'); 
grid on; legend('trq 1', 'trq 2');


%% parameters 

% Computer start/end pot energy
U_start = rrbotPotEnergy(apos(:,1));
U_end = rrbotPotEnergy(apos(:,end));
U_all = zeros(1, length(jnts));
DU = U_end - U_start;

for i = 1:length(jnts)
    U_all(i) = rrbotPotEnergy(apos(:,i));
end
% figure; plot(atime, U_all, 'r'); grid on;


% Do energy integration
W = 0;
W_all = zeros(1, length(jnts)-1);
DP = [0 0]';
for i = 1:(length(jnts)-1)
    tnow = jnts{i}.header.stamp.time;
    tnext = jnts{i+1}.header.stamp.time;
    dt = tnext - tnow;
    W = W + atrq(:,i)' * avel(:,i) * dt;
    W_all(i) = W;
    DP = DP + avel(:,i) * dt;
end

% plot W history
figure; plot(atime(1:end-1), W_all); grid on; 
title('Work history');

percent_error = (W - DU) / max(DU, W) * 100;
disp(['DU = ' num2str(DU)]);
disp(['W  = ' num2str(W)]);
disp(['E% = ' num2str(percent_error) '%']);










