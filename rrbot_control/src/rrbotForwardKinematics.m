function [ T01, T02 ] = rrbotForwardKinematics( q )
%RRBOTFORWARDKINEMATICS Compute FK for each joint pose

global width
global height2
global axel_offset

if length(q) ~= 2 
    error('Joint position should be length of 2');  
end

T01 = troty(q(1));
T12 = transl([0, width, height2-axel_offset*2]) * troty(q(2));
T02 = T01 * T12;

end

