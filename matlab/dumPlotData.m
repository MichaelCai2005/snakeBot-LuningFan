%% plot trajectory
f1 = figure;
hold on
for i=1:1:i_purtX
    for k=1:1:i_purtZ
        plot(xdata{i,k}(7,:), zdata{i,k}(7,:));
    end
end 
hold off

%% plot mean force
f2 = figure
xlabel('time (s)');
ylabel('F (N)');
hold on
ctn = 0;
fx=[];
fz=[];
for i=1:1:i_purtX-1
    for k=1:1:i_purtZ-1
        ctn = ctn + 1;
        fx(ctn,:) = fxPegsData{i,k}(1:15000);
        fz(ctn,:) = fzPegsData{i,k}(1:15000);
    end
end
fxMean = mean(fx);
fzMean = mean(fz);
figure
plot(fTimeData{1,1}(1:15000),fxMean,'k')
xlabel('Time (s)');
ylabel('Fx (N)');
hold off

%% plot mean force
f3 = figure
xlabel('time (s)');
ylabel('Fx (N)');
hold on
for i=1:1:i_purtX-1
    for k=1:1:i_purtZ-1
       plot(fTimeData{1,1}(1:15000),fxPegsData{i,k}(1:15000),'.k')
    end
end
