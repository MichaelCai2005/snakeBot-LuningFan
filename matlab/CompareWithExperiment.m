% KK = 13;    % needs to be the same as kNseg
% AA = 0.42 * pi / 2;
% ww = 0.15;
% stopTime = 10;
% RFTstartTime = 2.66; % 2.77 found by running 180 sec simulation
% kNseg = 13;
% lx = 0.0585;    % robot length
% ly = 0.035;     % robot height
% lz = 0.038;     % robot width
KK = 13.0;    % needs to be the same as kNseg
AA = 0.64;
ww = 0.15;
stopTime = 40;
RFTstartTime = 2.67; % 2.77 found by running 180 sec simulation
kNseg = 13;
lx = 0.056;    % robot length
ly = 0.035;     % robot height
lz = 0.038;     % robot width

dt=.03;
n_iter = 500;
integratorType = 1;
recoverySpeed = 0.6;
col_envelop = 0.03;
col_margin = 0.01;
multSin = 10 * 0.1;
multCos = 10 * 5.0;
load('C:\Users\Arman\Documents\Repos\Git\snake\matlab\ExperimentalData\experimentalRobot.mat');
[xs,zs,~] = runChrono(KK,AA,ww,0,0,0,stopTime,0,RFTstartTime,0,kNseg,lx,ly,lz,0,0, dt, n_iter, integratorType, recoverySpeed, col_envelop, col_margin,multSin,multCos);
segId = 7;
timeStartSim = floor(4.5/dt);
timeStartExp = 250;

delP1 = [];
delP2 = [];
tMinDiff = timeStartSim;
minSumDelP = 1e9;

range = floor(3 / dt);

for t=timeStartSim-range:timeStartSim+range
    delz = zs(segId,t) - xc(timeStartExp,segId);
    delx = xs(segId,t) - zc(timeStartExp,segId);
    
    delP1 = xs(1:13, t) - zc(timeStartExp,:)' - delx;
    delP2 = zs(1:13, t) - xc(timeStartExp,:)' - delz;
    delP = delP1.^2 + delP2.^2;
    sumDelP = sum(delP);
%                             fprintf('t %0.5e sumDelP %0.5e minSumDelP %0.5e \n',t, sumDelP, minSumDelP);
    if (sumDelP < minSumDelP)
        tMinDiff = t;
        minSumDelP = sumDelP;
    end
end
    
f1=figure
plot(xc(timeStartExp,:),zc(timeStartExp,:),'.-k');
delz = zs(segId,tMinDiff) - xc(timeStartExp,segId);
delx = xs(segId,tMinDiff) - zc(timeStartExp,segId);
hold on
% plot(zs(1:13,867), xs(1:13,867),'r');
plot(zs(1:13,tMinDiff) - delz, xs(1:13,tMinDiff) - delx,'.-r');
hold off

%% plot head over time
f2=figure
segment = 7;
% plot(xc(:,7),zc(:,7),'.-k');
plot(xc(:,segment),zc(:,segment),'.-k');
hold on
% plot(xc(:,11),zc(:,11),'ok');
timeEnd = size(xs,2);
delz = zs(segId,tMinDiff) - xc(timeStartExp,segId);
delx = xs(segId,tMinDiff) - zc(timeStartExp,segId);
% plot(zs(7,tMinDiff:timeEnd) - delz, xs(7,tMinDiff:timeEnd) - delx,'.-r');
plot(zs(segment,tMinDiff:timeEnd) - delz, xs(segment,tMinDiff:timeEnd) - delx,'.-r');
axis equal
% plot(zs(11,tMinDiff:timeEnd) - delz, xs(11,tMinDiff:timeEnd) - delx,'or');
hold off
%% animate
% for i=1:size(xc,1)
%     i
%     plot(xc(i,:),zc(i,:),'k'); 
%     xlim([-.2,.8]);
%     ylim([-.2,1.2]);
%     
%     drawnow;
% end
% 
% figure
% for i=800:size(xs,2)
%     i
%     plot(zs(1:13,i), xs(1:13,i),'r');
% %     xlim([-.2,.8]);
% %     ylim([-.2,1.2]);
%     
%     drawnow;
% end