%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% input parameters into code
%  1. KK -- wave parameter (close to number of segments)
%  2. AA -- angular amplitude**
%  3. ww -- temporal frequency
%  4. sp -- inner peg spacing**
%  5. wpx -- peg x position**
%  6. stopTime -- how long in seconds to run simulation
%  7. saveVideo -- 1 for yes, 0 for no
%  8. RFTstartTime -- when (in seconds) to turn RFT on (need to find best time to minimize initial z-translation.)
%  9. pegFriction -- friction between pegs and snake segments**
% 10. kNseg -- (2n-1) * (n = desired number of body segments)
% 11. lx = dimension of snake segment along direction of motion**
% 12. ly = dimension of snake segment out of screen
% 13. lz = dimension of snake segment in plane of motion, transverse to initial trajectory**
% 14. boxSx = peg diameter**
% 15. kNpeg = number of pegs
% 16. wpz -- peg z position (offset)**

% 17. dt -- array of time steps for parametric sweep
% 18. iter -- array of number of iterations for parametric sweep
% 19. timeIntegrationType -- (0): Anitescue, (1): Tasora, (2): HHT, (3):
%       Newmark, (4): RK45, (5): Trapezoidal, (6): Implicit Euler 
% 20. recoverySpeed -- contact recovery speed
% 21. colEnvelop -- collision envelop (outward)
% 22. colMargine -- collision margine (inward)

%** = iterate over these

% Indicies (Identifiers): 
%                       Robot: {0, 1, 2, ..., (N_Seg -1)}                    
%                       Robot Head and Tail: (N_Seg): Head, (N_Seg+1): Tail
%                       Ground: -1
%                       Pegs: -2, -3, -4, ....
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ./SnakeProject 25 0.6 0.2 0.14 1 10 0 1.70 0 25 0.2 0.1 0.1 0.1 51 1
KK = 13;    % needs to be the same as kNseg
AA = 0.64; %0.4 * pi / 2;
ww = 0.15;
sp = 0.061; % peg spacing
wpx = 1.5;
wpz = 0;
stopTime = 40;
saveVideo = 1;
RFTstartTime = 2.67; 
pegFriction = 0.4;
kNseg = 13;
lx = 0.056; % robot length
ly = 0.035;     % robot height
lz = 0.038;     % robot width
boxSx = 0.045;  % peg diameter
kNpeg = 0;%1;
multSin = 1.0; % 0.1 % soil model sin term prefactor
multCos = 50.0; % 5.0 % soil model cos term prefactor

dt = .01;%.002;
iter = 500;
timeIntegrationType = 1;
recoverySpeed =0.6;
colEnvelop = .01;
colMargine = 0;%.01;

% xzPeg = {xzPegSim1;xzPegSim2;xzPegSim3;xzPegSim4};
% xzPeg = {[ 0.0113,   0.5066];
%     [  0.0102,   0.5046];
%     [  0.0100 ,  0.5046];
%     [ 0.0099 ,  0.5046]};


segId = 7;
% load('C:\Users\Arman\Documents\Repos\Git\snake\matlab\ExperimentalData\expData4runs.mat');
load('C:\Users\Arman\Documents\Repos\Git\snake\matlab\ExperimentalData\10expRuns.mat');

%% first run to tune position
stopTime0 = 10;
kNpeg0 = 0;
[xs0,zs0,~] = runChrono(KK,AA,ww,0,0,0,stopTime0,0,RFTstartTime,0,kNseg,lx,ly,lz,boxSx,kNpeg0, dt, iter, timeIntegrationType, recoverySpeed, colEnvelop, colMargine,multSin,multCos);
%%
for i=1:10
    timeStartExp = 1;
    xc0 = data3{i,1}.xbody;
    zc0 = data3{i,1}.zbody;
    xc=xc0(:,[13 12 11 10 9 8 7 6 5 4 3 2 1]);
    zc=zc0(:,[13 12 11 10 9 8 7 6 5 4 3 2 1]);
    xcMean = mean(xc(timeStartExp,:))
    zcMean = mean(zc(timeStartExp,:))
    zc7 = zc(timeStartExp,segId);
    dXPeg = data3{i}.xpegs - xcMean
    dZPeg = data3{i}.zpegs - zcMean
    %% determine the start time for match
    timeStartSim = floor(5.5/dt);
    delP1 = [];
    delP2 = [];
    tMinDiff = timeStartSim;
    minSumDelP = 1e9;
    range = floor(3 / dt);
    for t=timeStartSim-range:timeStartSim+range
        delz = zs0(segId,t) - xc(timeStartExp,segId);
        delx = xs0(segId,t) - zc(timeStartExp,segId);
        delP1 = xs0(1:13, t) - zc(timeStartExp,1:13)' - delx;
        delP2 = zs0(1:13, t) - xc(timeStartExp,1:13)' - delz;
        delP = delP1.^2 + delP2.^2;
        sumDelP = sum(delP);
        if (sumDelP < minSumDelP)
            tMinDiff = t;
            minSumDelP = sumDelP;
        end
    end 
    %% plot initial config
%     f1 = figure
%     plot(xc(timeStartExp,1:13),zc(timeStartExp,1:13),'.-k');
%     delz = zs0(segId,tMinDiff) - xc(timeStartExp,segId);
%     delx = xs0(segId,tMinDiff) - zc(timeStartExp,segId);
%     hold on
%     plot(zs0(1:13,tMinDiff) - delz, xs0(1:13,tMinDiff) - delx,'.-r');
%     hold off   
    %% run sim
    xsMean = mean(xs0(1:13,tMinDiff));
    zsMean = mean(zs0(1:13,tMinDiff));
    xPegSim = xsMean + dZPeg;
    zPegSim = zsMean + dXPeg;
    system(['SnakeProject.exe ', num2str(KK), ' ', num2str(AA),  ' ', num2str(ww), ' ',...
    num2str(sp), ' ', num2str(wpx), ' ', num2str(stopTime),...
    ' ', num2str(saveVideo), ' ',num2str(RFTstartTime),' ',num2str(pegFriction),' ',...
    num2str(kNseg),' ',num2str(lx),' ',num2str(ly),' ',num2str(lz),' ',num2str(boxSx),' ',...
    num2str(kNpeg),' ', num2str(wpz),' ',...
    num2str(dt),' ', num2str(iter),' ',num2str(timeIntegrationType), ' ',...
    num2str(recoverySpeed),' ',num2str(colEnvelop),...
    ' ',num2str(colMargine), ' ', num2str(multSin), ' ', num2str(multCos), ...
    ' ', num2str(xPegSim), ' ', num2str(zPegSim)]);
    
    movdata = importdata('snake.mov');
    id = movdata(:,1); % identifies body segment
    numSegments = length(unique(id));%max(id)-min(id(id>=0))+1;
    x = reshape(movdata(:,2),numSegments,size(movdata,1)/numSegments);
    z = reshape(movdata(:,4),numSegments,size(movdata,1)/numSegments);
    mTime = reshape(movdata(:,15),numSegments,size(movdata,1)/numSegments);

    dxPegSimExp = xPegSim - data3{i}.zpegs;
    dzPegSimExp = zPegSim - data3{i}.xpegs;
    xdata{i} = x - dxPegSimExp;
    zdata{i} = z - dzPegSimExp;
    mTimeData{i} = mTime(1,:); % they are all the same
    tIdStartSim{i} = tMinDiff;
    id = reshape(id,numSegments,size(movdata,1)/numSegments);

    [fxSnake,fzSnake,fxPegs,fzPegs,idSnake,idPegs,fTime] = importForceData('','snake.cot',RFTstartTime,dt); 
    sprintf('simulation time %d\n', fTime);
    fxSnakeData{i}=fxSnake;
    fzSnakeData{i}=fzSnake;
    fxPegsData{i}=fxPegs;
    fzPegsData{i}=fzPegs;
    fTimeData{i}=fTime;
end

%% position
plotColorExp = {'.-g', '.-r', '.-b', '.-k','.-y'};
plotColorSim = {'-.g', '-.r', '-.b', '-.k','-.y'};
figurePositions = figure
hold on
for i=6:10
    xc0 = data3{i,1}.xbody;
    zc0 = data3{i,1}.zbody;
    xc=xc0(:,[13 12 11 10 9 8 7 6 5 4 3 2 1]);
    zc=zc0(:,[13 12 11 10 9 8 7 6 5 4 3 2 1]);
    plot(xc(:,segId),zc(:,segId),plotColorExp{i-5});
    plot(zdata{i}(segId,tMinDiff:end),xdata{i}(segId,tMinDiff:end),plotColorSim{i-5});
    axis equal
end
plot(data3{1}.xpegs, data3{1}.zpegs, '.m', 'MarkerSize',40);
print(figurePositions,'-r300','-djpeg',strcat('Trajectories', '.jpg'));  %'-dtiff ' for tiff 
saveas(figurePositions,strcat('Trajectories'),'fig');
hold off 

%% Force
pegSegmentPlot = 1;
for i=1:10
    f2 = figure
    %**** force Sim
    fx=fxPegsData{i}(pegSegmentPlot,:);
    fz=fzPegsData{i}(pegSegmentPlot,:);
    tStartSim = dt * tIdStartSim{i};
%     f2 = figure
    subplot(2,2,4)
    plot(fTimeData{i} - tStartSim,fx, 'k');
    xlabel('t (s)');
    ylabel('F_x (N), sim.');
    xlim([0 20]);
%     print(f2,'-r100','-djpeg ',strcat('forceSimX_', int2str(i) , '.jpg'));  %'-dtiff ' for tiff 
%     saveas(f2,strcat('forceSimX_', int2str(i)),'fig');
%     f3 = figure
    subplot(2,2,2)
    plot(fTimeData{i} - tStartSim,-fz, 'k');
    xlabel('t (s)');
    ylabel('F_z (N), sim.');
    xlim([0 20]);
%     print(f3,'-r100','-djpeg ',strcat('forceSimZ_', int2str(i) , '.jpg'));  %'-dtiff ' for tiff 
%     saveas(f3,strcat('forceSimZ_', int2str(i)),'fig');
    %**** force Experiment
    tShiftExperiment = -3.2;
%     f4 = figure
    subplot(2,2,1)
    plot(data3{i,1}.tf - tShiftExperiment, data3{i,1}.fx, 'k');
    xlabel('t (s)');
    ylabel('F_x (N), exp.');
    xlim([0 20]);
%     print(f4,'-r100','-djpeg ',strcat('forceExpX_', int2str(i) , '.jpg'));  %'-dtiff ' for tiff 
%     saveas(f4,strcat('forceExpX_', int2str(i)),'fig');
%     f5 = figure
    subplot(2,2,3)
    plot(data3{i,1}.tf - tShiftExperiment, data3{i,1}.fz, 'k');
    xlabel('t (s)');
    ylabel('F_z (N), exp.');
    xlim([0 20]);
%     print(f5,'-r100','-djpeg ',strcat('forceExpZ_', int2str(i) , '.jpg'));  %'-dtiff ' for tiff 
%     saveas(f5,strcat('forceExpZ_', int2str(i)),'fig');   

    % save them all
    print(f2,'-r300','-djpeg ',strcat('forces', int2str(i) , '.jpg'));  %'-dtiff ' for tiff 
    saveas(f2,strcat('forces', int2str(i)),'fig');
end
%%
% %% Force
% for i=1:1
%     tShiftExperiment = -3.2;
%     fx=fxPegsData{i}(pegSegmentPlot,:);
%     fz=fzPegsData{i}(pegSegmentPlot,:);
%     tStartSim = dt * tIdStartSim{i};
%     f2 = figure
%     hold on
%     plot(fTimeData{i} - tStartSim, -fz, '.k');
%     plot(data3{i,1}.tf - tShiftExperiment, data3{i,1}.fx, '.g');
%     hold off
%     xlabel('t (s)');
%     ylabel('F_x (N)');
%     print(f2,'-r300','-djpeg ',strcat('forceX_', int2str(i) , '.jpg'));  %'-dtiff ' for tiff 
%     saveas(f2,strcat('forcez_', int2str(i)),'fig');
%     f3 = figure
%     hold on
%     plot(fTimeData{i} - tStartSim, -fx, '.k');
%     plot(data3{i,1}.tf - tShiftExperiment, data3{i,1}.fz, '.g');
%     xlabel('t (s)');
%     ylabel('F_z (N)');
%     print(f3,'-r300','-djpeg ',strcat('forceZ_', int2str(i) , '.jpg'));  %'-dtiff ' for tiff 
%     saveas(f3,strcat('forceZ_', int2str(i)),'fig');
% end
%% save data
clear f2
clear figurePositions
save('dataForceOnPegTest.mat');
