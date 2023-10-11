function r0 = InitialPose(l,a, b)

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

[rMax,I_Max] = max(r_lowMax);
[rMin,I_min] = min(r_highMin);

r0 = (r_low(:,I_Max) + r_high(:,I_min))*(0.5);

end