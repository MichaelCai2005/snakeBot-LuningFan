function [fxSnake,fzSnake,fxPegs,fzPegs,idSnake,idPegs,realTime] = importForceData(directory,filename,RFTstartTime,dt,savePath,newName)

if nargin < 6 || isempty(newName)
    newName = filename(1:end-4);
end

forces = importdata([directory filename]);
allID = forces(:,1);
numID = length(unique(allID));

fx = reshape(forces(:,2),numID,size(forces,1)/numID);
%fy = reshape(forces(:,3),numID,size(forces,1)/numID);
fz = reshape(forces(:,4),numID,size(forces,1)/numID);
t = reshape(forces(:,5),numID,size(forces,1)/numID);
allID = reshape(allID,numID,size(forces,1)/numID);

id = allID(:,1);

pegs = find(id < -1);
snake = find(id > -1);

t1=t(1,:);
ind = find(t1>RFTstartTime-3 * dt & t1 < RFTstartTime + 3 * dt); 
tRFT = floor(mean(ind));


fxSnake = fx(snake,1:end);
fzSnake = fz(snake,1:end);
idSnake = id(snake);

fxPegs = fx(pegs,1:end);
fzPegs = fz(pegs,1:end);
idPegs = id(pegs);

realTime = t(1,1:end);


if nargin == 6 && ~isempty(savePath)
    save([savePath newName '_forces.mat'],'fxSnake','fzSnake','idSnake','fxPegs','fzPegs','idPegs','tRFT');
end
