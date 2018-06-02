% Zihan Chen 
% 2018-06-01 
% Verify Potential Energy

clc; clear; close all;
global width height2 height3 axel_offset m1 m2 com1 com2 g


% ========================
% Robot Params
% ========================
width = 0.1;
height2 = 1;
height3 = 1;
axel_offset = 0.05;

m1 = 1.0;
m2 = 1.0;

com1 = [0, 0, height2/2-axel_offset]';
com2 = [0, 0, height3/2-axel_offset]';
g = [0, 0, 9.81']';

X = [m1 m1*com1(1) m1*com1(2) m1*com1(3) ...
    m2 m2*com2(1) m2*com2(2) m2*com2(3)]';

% =========================
% Eq. from a bag file
% =========================

% bag_fname_list = {...
%     '2018-06-01-19-51-01.bag', ...
%     '2018-06-01-19-51-01.bag', ...
%     };

bag_fname_list = cell(20, 1);

for i = 1:20    
    fname = sprintf('~/.ros/rrbot-gc-%02d.bag', i-1);
    bag_fname_list{i} = fname;
end

AGC = zeros(length(bag_fname_list), length(X));
bGC = zeros(length(bag_fname_list), 1);
bGC_Real = zeros(length(bag_fname_list), 1);

for f = 1:length(bag_fname_list)
    
    % construct data points
    bag_fname = bag_fname_list{f};
    bag = ros.Bag(bag_fname);
    jnts = bag.readAll('/rrbot/joint_states');
    
    
    % compute Delta U
    q_start = jnts{1}.position;
    q_end = jnts{end}.position;
    U_start = rrbotPotEnergy(q_start);
    U_end = rrbotPotEnergy(q_end);
    bGC_Real(f,:) = U_end - U_start;
    
    [T01, T02] = rrbotForwardKinematics(q_start);
    DU_start = g' * [transl(T01) T01(1:3,1) T01(1:3,2) T01(1:3,3) ...
        transl(T02) T02(1:3,1) T02(1:3,2) T02(1:3,3)];
    
    [T01, T02] = rrbotForwardKinematics(q_end);
    DU_end = g' * [transl(T01) T01(1:3,1) T01(1:3,2) T01(1:3,3) ...
        transl(T02) T02(1:3,1) T02(1:3,2) T02(1:3,3)];
    DU = DU_end - DU_start;
    AGC(f,:) = DU;
    
    
    % Do energy integration
    W = 0;
    for i = 1:(length(jnts)-1)
        trq = jnts{i}.effort;
        vel = jnts{i}.velocity;
        
        tnow = jnts{i}.header.stamp.time;
        tnext = jnts{i+1}.header.stamp.time;
        dt = tnext - tnow;
        
        W = W + trq' * vel * dt;
    end    
    bGC(f,:) = W;
end


% Ax=b
X_est = pinv(AGC) * bGC;
X_estReal = AGC \ bGC_Real;




AGC = AGC










