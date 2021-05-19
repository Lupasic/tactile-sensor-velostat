% kx+b = y
%[x1 1]*[k;b] = y1
%[V1 1]*[k;b] = m1*g

fds = fileDatastore('futek_calibration.txt', 'ReadFcn', @importdata)
fullFileNames = fds.Files
numFiles = length(fullFileNames)
% Loop over all files reading them in and plotting them.
X = cell(numFiles,1);
for k = 1 : numFiles

    fprintf('Now reading file %s\n', fullFileNames{k});
    A = importdata(fullFileNames{k},' ');
    X{k} = A(:,:);  
end

for k = 1 : numFiles
    cur_plot = X{k};
    fitpoints = fit(cur_plot(:,1),cur_plot(:,2),'poly1','Robust','on')
    scatter(cur_plot(:,1),cur_plot(:,2))
    hold on
    plot(fitpoints)
end

% hold on
% plot(X(1),[X(1).^2 X(1) ones(1,1)]*[fit1.p1;fit1.p2;fit1.p3])
% hold on
% plot(fit1)
