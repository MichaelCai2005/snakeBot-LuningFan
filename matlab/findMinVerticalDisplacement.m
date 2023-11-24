function [rftStartTime,slope,dx,dz] = findMinVerticalDisplacement(KK,AA,ww,stopTime,kNseg,lx,ly,lz,t0)

%this runs the snake on a surface with zero pegs to determine what the best
%choice for rftStartTime is so that the snake goes straight

n_iter = 500;
integratorType = 1;
recoverySpeed = 0.6;
col_envelop = 0.03;
col_margin = 0.01;

dt = 0.01;
elementsPerT = round(1/ww/dt); % T = 1 / ww;  
startInd = 2*elementsPerT+1;
stopInd = floor(stopTime/dt/elementsPerT)*elementsPerT;

tMin = 0.0;


if nargin < 9 || isempty(t0)
    t0 = 2.0;
end
tMax = 2*t0;

% run the first time

[x,z,~] = runChrono(KK,AA,ww,0,0,0,stopTime,0,t0,0,kNseg,lx,ly,lz,0,0, dt, n_iter, integratorType, recoverySpeed, col_envelop, col_margin);
p0 = polyfit(mean(x(:,startInd:stopInd),1),mean(z(:,startInd:stopInd),1),1);


coarseDT = 0.1;
if p0(1,1) < 0
    % then tStart will be greater than t0
    testTimes = t0:coarseDT:tMax;
else testTimes = t0:-coarseDT:tMin;
end


p = zeros(length(testTimes),2);
p(1,:) = p0;

for i = 2:length(testTimes)
    [x,z,~] = runChrono(KK,AA,ww,0,0,0,stopTime,0,testTimes(i),0,kNseg,lx,ly,lz,0,0, dt, n_iter, integratorType, recoverySpeed, col_envelop, col_margin);
    p(i,:) = polyfit(mean(x(:,startInd:stopInd),1),mean(z(:,startInd:stopInd),1),1);
    if  sign(p(i,1))~= sign(p(i-1,1))
        break;
        
    end
end

fineDT = 0.01;

testTimes2 = testTimes(i-1):fineDT:testTimes(i);

p2 = zeros(length(testTimes2),2);
p2(1,:) = p(i-1,:);
p2(end,:) = p(i,:);

if isempty(testTimes2)
    testTimes2 = testTimes(i):fineDT:testTimes(i-1);
    p2 = zeros(length(testTimes2),2);
    p2(1,:) = p(i,:);
    p2(end,:) = p(i-1,:);
end
% order so that fewer runs are needed
if abs(p2(end,1)) < abs(p2(1,1))
    testTimes2 = fliplr(testTimes2);
    p2 = flipud(p2);
end



for i = 2:length(testTimes2)
    [x,z,~] = runChrono(KK,AA,ww,0,0,0,stopTime,0,testTimes2(i),0,kNseg,lx,ly,lz,0,0, dt, n_iter, integratorType, recoverySpeed, col_envelop, col_margin);
    p2(i,:) = polyfit(mean(x(:,startInd:stopInd),1),mean(z(:,startInd:stopInd),1),1);
    if  sign(p2(i,1))~= sign(p2(i-1,1))
        break;
    end
end

slopes = p2(i-1:i,1);
ts = testTimes2(i-1:i);
[~,ind] = min(abs(slopes));
slope = slopes(ind);
rftStartTime = ts(ind);


[~,ix] = max(x(:,round(rftStartTime/dt))); % head is the segment with the largest x
% find local extrema
[maxIdx,minIdx,maxVals,minVals] = findLocalMaxAndMin(z(ix,round(rftStartTime/dt):end),floor(1/ww/dt),0);

% find average x displacement for head to return to initial phase
 dx = mean([diff(x(ix,round(rftStartTime/dt)+maxIdx-1)) diff(x(ix,round(rftStartTime/dt)+minIdx-1))]);
 dz = mean(maxVals)-mean(minVals);
