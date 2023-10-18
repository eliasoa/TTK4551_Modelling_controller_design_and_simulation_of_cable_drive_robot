function r0 = InitialPose(l,a,b)
% r0 = InitialPose(l,a,b) calculates a initial pose for the 
% tool centrepoint (TCP) of a Cable-Driven Paralell robot
% using eq. (4.40) - (4.41) in Pott, p 143
%
%
%
%
% Author: Magnus Grøterud
% Date: Kven kveit
% Revision:

% Memory Allocation
r_low = zeros(2,4);
r_high = zeros(2,4);


m = 4;

for i = 1:m
    r_low(:,i) = a(:,i) - (l(i) + norm(b(:,i))*[1;1]);
    r_high(:,i) = a(:,i) + (l(i) + norm(b(:,i))*[1;1]);
end

% Intersection of all m bounding boxes

% Cursed
r_lowMax = max(r_low);
r_highMin = min(r_high);

[~,I_Max] = max(r_lowMax);
[~,I_min] = min(r_highMin);

r0 = (r_low(:,I_Max) + r_high(:,I_min))*(0.5);

end