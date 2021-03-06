function [total_peaks_rmse] = common_postproc_plots(exp_futek_data,exp_velostat_data,title_name, sensor_names)
total_peaks_rmse = zeros(size(exp_futek_data,1),1);

for i = 1:size(exp_futek_data,1)
    xa = cell2mat(exp_futek_data{i,1}(:,2));
    ya = cell2mat(exp_futek_data{i,1}(:,1));
    xb = cell2mat(exp_velostat_data{i,1}(:,2));
    yb = cell2mat(exp_velostat_data{i,1}(:,1));
    nya = norm_data(cell2mat(exp_futek_data{i,1}(:,1)));
    nyb = norm_data(cell2mat(exp_velostat_data{i,1}(:,1)));
    
    figure
    sgtitle(title_name + "sensor " + sensor_names(i))
    
    subplot(2,2,1)
    cur_plot = plot(xa,ya);
    title("Futek");
    xlabel("Timestamp, ms");
    ylabel("Force, N");
    cur_plot(1).LineWidth = 2;
    
    subplot(2,2,2)
    cur_plot = plot(xb,yb);
    title("Velostat");
    xlabel("Timestamp, ms");
    ylabel("Force, units");
    cur_plot(1).LineWidth = 2;
    cur_plot(1).MarkerSize= 4;
    cur_plot(1).Marker = "*";


    subplot(2,2,[3,4])
    cur_plot = plot(xa,nya,xb,nyb);
    title("Sensors comparison, normalized units");
    xlabel("Timestamp, ms");
    ylabel("Normalized force, units");
    legend("Futek","Velostat");
    cur_plot(1).LineWidth = 2;
    cur_plot(2).LineWidth = 2;
    cur_plot(2).LineStyle = '--';
    cur_plot(2).MarkerSize= 4;
    cur_plot(2).Marker = "*";
    
    futek_peaks = findpeaks(nya,"MinPeakDistance",150);
    velostat_peaks = findpeaks(nyb,"MinPeakDistance",50);
%     "MinPeakHeight",40, "MinPeakProminence",4
    if length(futek_peaks) == length(velostat_peaks) && ~isempty(futek_peaks)
        total_peaks_rmse(i) = rms(futek_peaks - velostat_peaks);
        disp(total_peaks_rmse(i) + " Root Mean Square Error")
    else
        disp(length(futek_peaks) + " amount of futek peaks")
        disp(length(velostat_peaks) + " amount of velostat peaks")
        disp("Amount of peaks are not equal or equal to 0")
        total_peaks_rmse(i) = NaN;
    end
end
end

