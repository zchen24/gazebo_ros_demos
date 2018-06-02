function [ U ] = rrbotPotEnergy( q )
%RRBOTPOTENERGY Compute RRBot potential energy
%   U = rrbotPotEnergy(q), q is the joint position, returns rrbot potential
%   energy

if length(q) ~= 2 
    error('Joint position should be length of 2');  
end

width = 0.1;
height2 = 1;
height3 = 1;
axel_offset = 0.05;

m1 = 1.0;
m2 = 1.0;

com1 = [0, 0, height2/2-axel_offset]';
com2 = [0, 0, height3/2-axel_offset]';
g = [0, 0, 9.81']';

T01 = troty(q(1));
T12 = transl([0, width, height2-axel_offset*2]) * troty(q(2));


% compute start / end potential energy 
U1 = g' * (transl(T01) + t2r(T01) * com1) * m1;

T02 = T01 * T12;
U2 = g' * (transl(T02) + t2r(T02) * com2) * m2;

U = U1 + U2;

end

