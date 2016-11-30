
via = [0 10 20 30 40]';
times = [0 5 10 15 20]';

tRes = 0.01;


[viaV, dur] = Planner(via,times);

viaV = viaV';
dur = dur';

% t1 = 0:0.01:4.99;
% t1 = t1';
% t2 = 5:0.01:9.99;
% t2 = t2';
% t3 = 10:0.01:14.99;
% t3 = t3';
% t4 = 15:0.01:20;
% t4 = t4';

for i = 1:(size(via)-1)
     tx = times(i):tRes:(times(i+1)-tRes);
     tx = tx';
     ti(:,i) = tx;
    [posArray(i), velArray(i), accelArray(i)] = PolySegment(via(i),via(i+1),viaV(i),viaV(i+1));
    
end
for i = 1:(size(via)-1)
    posIFun = matlabFunction(posArray(i));
    [m,n] = size(ti);
    for j = 1:m
        position1(j) = feval(posIFun,(ti(j,i)/5));
    end
end



% Tau stuff




