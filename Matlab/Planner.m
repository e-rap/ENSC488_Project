function [ viaV, dur ] = Planner( via, times )
%PLANNER Summary of this function goes here
%   Detailed explanation goes here

viaV(1) = 0;
for i = 2:size(times)
    diffpos = via(i) - via(i-1);
    dur(i-1) = times(i) - times(i-1);
    viaV(i) = diffpos/dur(i-1);
end
viaV(size(via)) = 0;
end
