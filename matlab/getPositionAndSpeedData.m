function [xSnake,zSnake,vxSnake,vzSnake,t,idSnake,xPegs,zPegs,idPegs] = getPositionAndSpeedData(directory,filename,tRFT,savePath,newName)

% filename is the full path of a .mov file

if nargin < 5 || isempty(newName)
    newName = filename(1:end-4);
end




data = importdata([directory filename]);
dt = 0.01;
% coordinate system:
% +x = direction unperturbed snake would move
% +y = out of page: this is not needed here -- snake confined to xz plane, so these forces should be orders of magnitude smaller than x or z
% +z = to the left

%file format: segment ID; Position x, y, z; Velocity vx, vy, vz; Quarternion Qx, Qy, Qz, Qw, Angular velocity wx, wy, wz

% values are given for segment/peg specified
id = data(:,1); % identifies body segment
numSegments = length(unique(id));%max(id)-min(id(id>=0))+1;

t = dt:dt:(length(id)/numSegments*dt);

% matrices with each column corresponding to a segment and
% each row representing a point in time
x = reshape(data(:,2),numSegments,size(data,1)/numSegments); 
z = reshape(data(:,4),numSegments,size(data,1)/numSegments);

vx = reshape(data(:,5),numSegments,size(data,1)/numSegments); 
vz = reshape(data(:,7),numSegments,size(data,1)/numSegments);

id  = reshape(id,numSegments,size(data,1)/numSegments);

id = id(:,1);

% -1 is the ground
pegs = find(id < -1);
snake = find(id > -1);

xSnake = x(snake,tRFT:end);
zSnake = z(snake,tRFT:end);
vxSnake = vx(snake,tRFT:end);
vzSnake = vz(snake,tRFT:end);
idSnake = id(snake);

xPegs = x(pegs,1);
zPegs = z(pegs,1);
idPegs = id(pegs);

t = t(tRFT:end);

if nargin == 5 && ~isempty(savePath)
    save([savePath newName '_positions.mat'],'xSnake','zSnake','vxSnake','vzSnake','t','idSnake','xPegs','zPegs','idPegs','tRFT');
end

