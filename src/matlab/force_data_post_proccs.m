% should add out folder to matlab path
fds = fileDatastore('futek_data/experiment_*.txt', 'ReadFcn', @importdata);
fullFileNames = fds.Files;
% [pathstr, name, ext] = fileparts(fullFileNames{1})
numFiles = length(fullFileNames);
% Loop over all files reading them in and plotting them.
X = cell(numFiles,1);
for k = 1 : numFiles

    fprintf('Now reading file %s\n', fullFileNames{k});
    A = importdata(fullFileNames{k},' ');
    if isempty(A) == false
        X{k} = A(:, 1);
    end
end
exp_date= datestr(now,31);
fileID = fopen(['results/experiment_results_',exp_date,'.txt'],'w');

for k = 1:numFiles
   [curpathstr, curname, curext] = fileparts(fullFileNames{k});
   fprintf(fileID,'%s\n',curname);
end

fprintf(fileID,'%s %12s %12s %12s\n','Name','avg','std','median');
% all_experiment_data = zeros(tt,numFiles);


tt = 100;
array_size = 10;
error = 3;


for k = 1 : numFiles
    cur_plot = X{k};
    % remove from array the values which not change for a long time
    for i=1:length(cur_plot)
        if length(cur_plot) - i < array_size
            break
        end
        cur_subplot=cur_plot(i:i+array_size-1);
        if isempty(cur_subplot(cur_subplot>cur_plot(i)+error)) && isempty(cur_subplot(cur_subplot<cur_plot(i)-error))
            cur_plot(i:i+array_size)=[];
        end
    end

    cur_avg = mean(cur_plot);
    cur_std = std(cur_plot);
    cur_qua = quantile(cur_plot,[0.25 0.50 0.75]);
    %rejecting outliers
    koeff = 3; %far out
    outlier_below = cur_plot<(cur_qua(1)-koeff*(cur_qua(3)-cur_qua(1)));
    outlier_upper = cur_plot>(cur_qua(3)+koeff*(cur_qua(3)-cur_qua(1)));
    cur_plot(logical(outlier_below+outlier_upper))=[];
    
    
    subplot(numFiles/3,3,k);
    xlabel("Amount of points")
    ylabel("Value")
    title(['Experiment ',num2str(k)])
    scatter(1:length(cur_plot),cur_plot)
    hold on
    plot([1,length(cur_plot)],[cur_avg,cur_avg])
    hold on
    plot([1,length(cur_plot)],[cur_qua(3),cur_qua(3)])
    hold on
    plot([1,length(cur_plot)],[cur_qua(1),cur_qua(1)])
    hold on
    plot([1,length(cur_plot)],[cur_qua(2),cur_qua(2)])
    
    legend("Points","Average value","0.75 quantile","0.25 quantile","Median")
    fprintf(fileID,'Exp. %i %12.3f %12.3f %10.3f\n',k,cur_avg, cur_std, cur_qua(2));
    
    
end
fclose(fileID);
savefig(['results/all_data_',exp_date,'.fig'])
% figure;
% boxplot(all_experiment_data)
% savefig(['results/box_chart_',exp_date,'.fig'])