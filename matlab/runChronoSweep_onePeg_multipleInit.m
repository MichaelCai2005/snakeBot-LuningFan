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
%% clear memory
prompt = 'should I clear memory? ';
inpCom = input(prompt,'s');
inpCom2 = upper(inpCom);
if (strcmp(inpCom2,'YES') | strcmp(inpCom2,'Y'))
    clear all
end
%%
Linux = 0;
%%
KK = 13;    % needs to be the same as kNseg
AA = 0.64; %0.4 * pi / 2;
ww = 0.15;
sp = 0.061; % peg spacing
wpx = 1.5;
wpz = 0;
stopTime = 40;
saveVideo = 0;
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

runCommand = ' ';
if (Linux == 0) 
    load('C:\Users\Arman\Documents\Repos\Git\snake\matlab\ExperimentalData\positionAndForceDataOnPeg.mat');
    runCommand = 'SnakeProject.exe ';
else
    load('/home/pazouki/chronoStuff/snake/matlab/ExperimentalData/positionAndForceDataOnPeg.mat');
    % Save library paths
    MatlabPath = getenv('LD_LIBRARY_PATH');
    % Make Matlab use system libraries
    setenv('LD_LIBRARY_PATH',getenv('PATH'))
    runCommand = './SnakeProject ';
end

xMe = mean(xc(1,:));
zMe = mean(zc(1,:));
dXe = xpegs - xMe;
dZe = zpegs - zMe;

xMs13=0.703335678769231;     % CM, 15 components
zMs13=0.010927415800000;
xMs=xMs13;     % CM
zMs=zMs13;

% xOnePeg = 1.361887876666667; 
% zOnePeg = -0.004578152082000; 
% xOnePeg = xMs + dZe; 
% zOnePeg = zMs + dXe; 
xOnePeg = xMs + zpegs; 
zOnePeg = zMs + xpegs;

% xPurturbPeg = [-1.75:0.05:.25] * boxSx;
% zPurturbPeg = [-3.25:0.1:.75] * boxSx;
% xPurturbPeg = [-.2:0.01:.2] * boxSx;
% zPurturbPeg = [-.2:0.01:.2] * boxSx;
% xPurturbPeg = [-.05] * boxSx;
% xPurturbPeg = [-1] * boxSx;
xPurturbPeg = [0.16:0.011:.58];%
zPurturbPeg = [-0.14:.011:0.14];%-0.009;

xdata={};
zdata={};
mTimeData={};
fxSnakeData={};
fzSnakeData={};
fxPegsData={};
fzPegsData={};
fTimeData={};
myLegend={};
myTitle={};
'yo1'
xOnePeg + xPurturbPeg(1)
'yo2'
zOnePeg + zPurturbPeg(1)
% for dt=[.1,.01,.003,.001,.0003,.0001]
for i_purtX=1:size(xPurturbPeg,2)
    i_purtX
    for i_purtZ=1:size(zPurturbPeg,2)
        tic
        system([runCommand, num2str(KK), ' ', num2str(AA),  ' ', num2str(ww), ' ',...
            num2str(sp), ' ', num2str(wpx), ' ', num2str(stopTime),...
            ' ', num2str(saveVideo), ' ',num2str(RFTstartTime),' ',num2str(pegFriction),' ',...
            num2str(kNseg),' ',num2str(lx),' ',num2str(ly),' ',num2str(lz),' ',num2str(boxSx),' ',...
            num2str(kNpeg),' ', num2str(wpz),' ',...
            num2str(dt),' ', num2str(iter),' ',num2str(timeIntegrationType), ' ',...
            num2str(recoverySpeed),' ',num2str(colEnvelop),...
            ' ',num2str(colMargine), ' ', num2str(multSin), ' ', num2str(multCos), ...
            ' ', num2str(xOnePeg + xPurturbPeg(i_purtX)), ' ', num2str(zOnePeg + zPurturbPeg(i_purtZ))]);
        fTime = toc
        sprintf('simulation time %d\n', fTime);

        movdata = importdata('snake.mov');
        id = movdata(:,1); % identifies body segment
        numSegments = length(unique(id));%max(id)-min(id(id>=0))+1;
        x = reshape(movdata(:,2),numSegments,size(movdata,1)/numSegments);
        z = reshape(movdata(:,4),numSegments,size(movdata,1)/numSegments);
        mTime = reshape(movdata(:,15),numSegments,size(movdata,1)/numSegments);

        xdata{i_purtX,i_purtZ} = x - xPurturbPeg(i_purtX);
        zdata{i_purtX,i_purtZ} = z - zPurturbPeg(i_purtZ);
        mTimeData{i_purtX,i_purtZ} = mTime(1,:); % they are all the same
        id = reshape(id,numSegments,size(movdata,1)/numSegments);

        [fxSnake,fzSnake,fxPegs,fzPegs,idSnake,idPegs,fTime] = importForceData('','snake.cot',RFTstartTime,dt); 
        fxSnakeData{i_purtX,i_purtZ}=fxSnake;
        fzSnakeData{i_purtX,i_purtZ}=fzSnake;
        fxPegsData{i_purtX,i_purtZ}=fxPegs;
        fzPegsData{i_purtX,i_purtZ}=fzPegs;
        fTimeData{i_purtX,i_purtZ}=fTime;
    end
end

%% plot positions
% snakeSegmentPlot = 7; %kNseg; 
% pegSegmentPlot = 1;
% blue=[0 0 1],;
% green=[0 1 0]
% red=[1 0 0];
% f1=figure
% hold on
% sizeXPurt = size(xPurturbPeg,2);
% sizeZPurt = size(zPurturbPeg,2);
% for i_purtX=1:sizeXPurt
%     for i_purtZ=1:sizeZPurt
%         mColor = green;
%         cC = i_purtX/sizeXPurt;
%         if (cC < 0.5)
%             mColor = 2 * ((0.5 - cC) * blue + cC * green);
%         else
%             mColor = 2 * ((1 - cC) * green + (cC - 0.5) * red);
%         end
%         moveTime = mTimeData{i_purtX,i_purtZ};
%         indMoveTime = find(moveTime > RFTstartTime-2*dt & moveTime < RFTstartTime+2*dt);
%         tStartInd=floor(mean(indMoveTime));
%         plot(zdata{i_purtX,i_purtZ}(snakeSegmentPlot,tStartInd:end),xdata{i_purtX,i_purtZ}(snakeSegmentPlot,tStartInd:end),'Color',mColor);
%     end
% end
% print(f1,'-r300','-djpeg ',strcat('pos_', '.jpg'))  %'-dtiff ' for tiff
% saveas(f1,'pos_all','fig');
% hold off
%% plot forces
% nameCount=0;
% for i_purtX=1:sizeXPurt
%     for i_purtZ=1:sizeZPurt
%         nameCount = nameCount + 1;
%         % ***** total force
%         f2=figure
%         hold on
%         fx=fxPegsData{i_purtX,i_purtZ}(pegSegmentPlot,:);
%         fz=fzPegsData{i_purtX,i_purtZ}(pegSegmentPlot,:);
%         f=sqrt(fx.^2+fz.^2);
%         plot(fTimeData{i_purtX,i_purtZ},f);
% %                     legend(myLegend);
% %                     title(myTitle{i_iter,i_tIntType,i_recSpeed,i_env,i_mar});
%         xlabel('t (s)');
%         ylabel('F (N)');
%         print(f2,'-r300','-djpeg ',strcat('force_', int2str(nameCount) , '.jpg'));  %'-dtiff ' for tiff 
%         saveas(f2,strcat('force_', int2str(nameCount)),'fig');
%         hold off
%     end
% end
%% crop position data from rft start time
xdata2 = {};
zdata2 = {};
for i_purtX=1:size(xPurturbPeg,2)
    for i_purtZ=1:size(zPurturbPeg,2)
        moveTime = mTimeData{i_purtX,i_purtZ};
        indMoveTime = find(moveTime > RFTstartTime-2*dt & moveTime < RFTstartTime+2*dt);
        tStartInd=floor(mean(indMoveTime))
        xdata2{i_purtX,i_purtZ} = xdata{i_purtX,i_purtZ}(:,tStartInd:end);
        zdata2{i_purtX,i_purtZ} = zdata{i_purtX,i_purtZ}(:,tStartInd:end);
    end
end
%%
save('dataBatchPeg.mat');
%% figuring out the initial position of the robot at rftStartTime
% xMs = mean(xdata2{1,1}(1:13,1))
% zMs = mean(zdata2{1,1}(1:13,1))
%%
if (Linux == 1)
% Reassign old library paths
    setenv('LD_LIBRARY_PATH',MatlabPath)
end


