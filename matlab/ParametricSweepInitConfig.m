% function [rftStartTime,slope,dx,dz] = ParametricSweepInitConfig(KK,AA,ww,RFTstartTime,lx,multSin,multCos)
% function out = ParametricSweepInitConfig()
load('/home/arman/Repos/Git/snake/matlab/ExperimentalData/experimentalRobot_7thComp_swapped.mat');
load('/home/arman/Repos/Git/snake/matlab/ExperimentalData/experimentalRobot.mat');
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

minSumDelP1 = 1e9;
KK_best1 = 13.0;
AA_best1 = .65;
ww_best1 = 0.15;
lx_best1 = 0.52;
multSin_best1 = 0;
RFTstartTimeBest1 = 2.7;
xs_best1 = [];
zs_best1 = [];
tStartIdx1 = timeStartSim;

minSumDelP2 = 1e9;
KK_best2 = 13.0;
AA_best2 = .65;
ww_best2 = 0.15;
lx_best2 = 0.52;
multSin_best2 = 0;
RFTstartTimeBest2 = 2.7;
xs_best2 = [];
zs_best2 = [];
tStartIdx2 = timeStartSim;

minSumDelP3 = 1e9;
KK_best3 = 13.0;
AA_best3 = .65;
ww_best3 = 0.15;
lx_best3 = 0.52;
multSin_best3 = 0;
RFTstartTimeBest3 = 2.7;
xs_best3 = [];
zs_best3 = [];
tStartIdx3 = timeStartSim;
segId = 7;

for KK = 12.5:0.5:13.5
    for AA = .58:.03:.7
        for ww = .14:.01:.16
            for lx = .05:.002:.058
                for multSin = 0:0.1:0.1
                    for RFTstartTime = 2.6:.05:2.8
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
                        %% best init condition
                         if (sumDelP_InitPos < minSumDelP1)
                             minSumDelP1 = sumDelP_InitPos;
                             tStartIdx1 = tMinDiff_InitPos;                  
                             KK_best1 = KK;
                             AA_best1 = AA;
                             ww_best1 = ww;
                             lx_best1 = lx;
                             multSin_best1 = multSin;
                             RFTstartTimeBest1 = RFTstartTime;
                             xs_best1 = xs - delx_best;
                             zs_best1 = zs - delz_best;
                         end
                        %% best dynamics
                        sizeX = size(xs,2);
                        xsTrimmed = xs(segId, tMinDiff_InitPos:sizeX) - delx_best;
                        zsTrimmed = zs(segId, tMinDiff_InitPos:sizeX) - delz_best;

                        zeInterpolate = interp1(xe7,ze7,xsTrimmed);
                        delZ_dyn = zsTrimmed - zeInterpolate;
                        sumDelZ = sum(delZ_dyn.^2);
                         if (sumDelZ < minSumDelP2)
                             minSumDelP2 = sumDelZ;                 
                             KK_best2 = KK;
                             AA_best2 = AA;
                             ww_best2 = ww;
                             lx_best2 = lx;
                             multSin_best2 = multSin;
                             RFTstartTimeBest2 = RFTstartTime;
                             xs_best2 = xs - delx_best;
                             zs_best2 = zs - delz_best;
                             tStartIdx2 = tMinDiff_InitPos;
                         end
                        %% best overall
                        sumTotal = sumDelZ + 1000 * sumDelP_InitPos
                        fprintf('rft %f sumInitPos %f sumDynamic %f sumTotal %f \n',RFTstartTime, sumDelP_InitPos, sumDelZ, sumTotal);
                        if (sumTotal < minSumDelP3)
                            minSumDelP3 = sumTotal;                 
                            KK_best3 = KK;
                            AA_best3 = AA;
                            ww_best3 = ww;
                            lx_best3 = lx;
                            multSin_best3 = multSin;
                            RFTstartTimeBest3 = RFTstartTime;
                            xs_best3 = xs - delx_best;
                            zs_best3 = zs - delz_best;
                            tStartIdx3 = tMinDiff_InitPos;
                        end
                    end
                end
            end
        end
    end
end   

%% fine tune
for KK = KK_best3-0.3:0.2:KK_best3+0.3
    for AA = AA_best3-.01:.02:AA_best3+.01
        for ww = ww_best3-.003:.002:ww_best3+.003
            for lx = lx_best3-.001:.0005:lx_best3+.001
                for multSin = multSin_best3
                    for RFTstartTime = RFTstartTimeBest3-.025:.005:RFTstartTimeBest3+.025
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
                        %% best init condition
                         if (sumDelP_InitPos < minSumDelP1)
                             minSumDelP1 = sumDelP_InitPos;
                             tStartIdx1 = tMinDiff_InitPos;                  
                             KK_best1 = KK;
                             AA_best1 = AA;
                             lx_best1 = lx;
                             RFTstartTimeBest1 = RFTstartTime;
                             xs_best1 = xs - delx_best;
                             zs_best1 = zs - delz_best;
                         end
                        %% best dynamics
                        sizeX = size(xs,2);
                        xsTrimmed = xs(segId, tMinDiff_InitPos:sizeX) - delx_best;
                        zsTrimmed = zs(segId, tMinDiff_InitPos:sizeX) - delz_best;

                        zeInterpolate = interp1(xe7,ze7,xsTrimmed);
                        delZ_dyn = zsTrimmed - zeInterpolate;
                        sumDelZ = sum(delZ_dyn.^2);
                         if (sumDelZ < minSumDelP2)
                             minSumDelP2 = sumDelZ;                 
                             KK_best2 = KK;
                             AA_best2 = AA;
                             lx_best2 = lx;
                             RFTstartTimeBest2 = RFTstartTime;
                             xs_best2 = xs - delx_best;
                             zs_best2 = zs - delz_best;
                             tStartIdx2 = tMinDiff_InitPos;
                         end
                        %% best overall
                        sumTotal = sumDelZ + 1000 * sumDelP_InitPos
                        fprintf('rft %f sumInitPos %f sumDynamic %f sumTotal %f \n',RFTstartTime, sumDelP_InitPos, sumDelZ, sumTotal);
                        if (sumTotal < minSumDelP3)
                            minSumDelP3 = sumTotal;                 
                            KK_best3 = KK;
                            AA_best3 = AA;
                            lx_best3 = lx;
                            RFTstartTimeBest3 = RFTstartTime;
                            xs_best3 = xs - delx_best;
                            zs_best3 = zs - delz_best;
                            tStartIdx3 = tMinDiff_InitPos;
                        end
                    end
                end
            end
        end
    end
end 
%%
fprintf('KK_best1 %0.5e \n',KK_best1);
fprintf('AA_best1 %0.5e \n',AA_best1);
fprintf('ww_best1 %0.5e \n',ww_best1);
fprintf('lx_best1 %0.5e \n',lx_best1);
fprintf('multSin_best1 %0.5e \n',multSin_best1);
fprintf('tStartIdx1 %0.5e \n',tStartIdx1);
fprintf('RFTstartTimeBest1 %0.5e \n',RFTstartTimeBest1);

fprintf('KK_best2 %0.5e \n',KK_best2);
fprintf('AA_best2 %0.5e \n',AA_best2);
fprintf('ww_best2 %0.5e \n',ww_best2);
fprintf('lx_best2 %0.5e \n',lx_best2);
fprintf('multSin_best2 %0.5e \n',multSin_best2);
fprintf('tStartIdx2 %0.5e \n',tStartIdx2);
fprintf('RFTstartTimeBes2 %0.5e \n',RFTstartTimeBest2);

fprintf('KK_best3 %0.5e \n',KK_best3);
fprintf('AA_best3 %0.5e \n',AA_best3);
fprintf('ww_best3 %0.5e \n',ww_best3);
fprintf('lx_best3 %0.5e \n',lx_best3);
fprintf('multSin_best3 %0.5e \n',multSin_best3);
fprintf('tStartIdx3 %0.5e \n',tStartIdx3);
fprintf('RFTstartTimeBest3 %0.5e \n',RFTstartTimeBest3);
                      
fileID = fopen('params.txt','w');
fprintf(fileID,'KK_best1 %0.5e \n',KK_best1);
fprintf(fileID,'AA_best1 %0.5e \n',AA_best1);
fprintf(fileID,'ww_best1 %0.5e \n',ww_best1);
fprintf(fileID,'lx_best1 %0.5e \n',lx_best1);
fprintf(fileID,'multSin_best1 %0.5e \n',multSin_best1);
fprintf(fileID,'tStartIdx1 %0.5e \n',tStartIdx1);
fprintf(fileID,'RFTstartTimeBest1 %0.5e \n',RFTstartTimeBest1);

fprintf(fileID,'KK_best2 %0.5e \n',KK_best2);
fprintf(fileID,'AA_best2 %0.5e \n',AA_best2);
fprintf(fileID,'ww_best2 %0.5e \n',ww_best2);
fprintf(fileID,'lx_best2 %0.5e \n',lx_best2);
fprintf(fileID,'multSin_best2 %0.5e \n',multSin_best2);
fprintf(fileID,'tStartIdx2 %0.5e \n',tStartIdx2);
fprintf(fileID,'RFTstartTimeBes2 %0.5e \n',RFTstartTimeBest2);

fprintf(fileID,'KK_best3 %0.5e \n',KK_best3);
fprintf(fileID,'AA_best3 %0.5e \n',AA_best3);
fprintf(fileID,'ww_best3 %0.5e \n',ww_best3);
fprintf(fileID,'lx_best3 %0.5e \n',lx_best3);
fprintf(fileID,'multSin_best3 %0.5e \n',multSin_best3);
fprintf(fileID,'tStartIdx3 %0.5e \n',tStartIdx3);
fprintf(fileID,'RFTstartTimeBest3 %0.5e \n',RFTstartTimeBest3);

f1=figure
plot(ze(:,timeStartExp),xe(:, timeStartExp),'.-k');
hold on
plot(zs_best3(1:13,tStartIdx3),xs_best3(1:13,tStartIdx3),'.-r');
hold off

%% plot head over time
f2=figure
plot(ze7,xe7,'.-k');
timeEnd = size(xs_best3,2);
hold on
plot(zs_best3(7,tStartIdx3:timeEnd),xs_best3(7,tStartIdx3:timeEnd),'.-r');
hold off
%% delete descending components, required for interpolation
% i = 2;
% while (i < size(xe7,1))
%     'yo0'
%     while ((xe7(i) <= xe7(i-1)) & (i < size(xe7,1)))
%         xe7(i) = [];
%         ze7(i) = [];
%     end
%     fprintf('(2) x %f %f \n', xe7(i), xe7(i-1));
%     i = i+1;
% end


