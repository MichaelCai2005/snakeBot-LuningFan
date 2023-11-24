function diffCurvesCumul = CurvesDrift(KK,AA,ww,lx,multSin,RFTstartTime)
% KK=13;
% AA=.6912;
% ww=0.15;
% lx=.0585;
% multSin=0;
% RFTstartTime=2.72;


load('experimentalRobot_7thComp_swapped.mat');
load('experimentalRobot.mat');
xe = zc';
ze = xc';

multCos = 5;    
kNseg = 13;
stopTime = 35;
ly = 0.035;     % robot height
lz = 0.038;     % robot width

dt=.03;
timeStartSim = floor(7 / dt); % 4.5(s) / delta_time
tSpan = floor(4 / dt);
timeStartExp = 250;

n_iter = 500;
integratorType = 1;
recoverySpeed = 0.6;
col_envelop = 0.03;
col_margin = 0.01;

segId = 7;

[xs,zs,~] = runChrono(KK,AA,ww,0,0,0,stopTime,0,RFTstartTime,0,kNseg,lx,ly,lz,0,0, dt, n_iter, integratorType, recoverySpeed, col_envelop, col_margin, multSin, multCos);
sumDelP_InitPos = 1e9;
tMinDiff_InitPos = 3;
delx_best = 0;
delz_best = 0;
for tStep=timeStartSim-tSpan:timeStartSim+tSpan
    delz = zs(segId,tStep) - ze(7,timeStartExp);
    delx = xs(segId,tStep) - xe(7,timeStartExp);

    delP1 = xs(1:13, tStep) - xe(:, timeStartExp) - delx;
    delP2 = zs(1:13, tStep) - ze(:, timeStartExp) - delz;
    delP = delP1.^2 + delP2.^2;
    sumDelP = sum(delP);
    if (sumDelP < sumDelP_InitPos) 
        sumDelP_InitPos = sumDelP;
        tMinDiff_InitPos = tStep;
        delx_best = delx;
        delz_best = delz;
    end
end
%% best dynamics
sizeX = size(xs,2);
xsTrimmed = xs(segId, tMinDiff_InitPos:sizeX) - delx_best;
zsTrimmed = zs(segId, tMinDiff_InitPos:sizeX) - delz_best;

zeInterpolate = interp1(xe7,ze7,xsTrimmed);
delZ_dyn = zsTrimmed - zeInterpolate;
sumDelZ = sum(delZ_dyn.^2);
%% best overall
sumTotal = sumDelZ + 1000 * sumDelP_InitPos;
diffCurvesCumul = sumTotal;

fprintf('sumDelP_InitPos %f sumDelZ %f sumTotal %f\n',sumDelP_InitPos, sumDelZ, sumTotal);


