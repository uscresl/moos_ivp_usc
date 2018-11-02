close all; 
clear all;

% import raw data
unmappeddat = importdata('data/data2018-04-27-12-23-31.txt');
str = string(unmappeddat(1));
A = strsplit(str, ' ');
for i=2:size(unmappeddat)
   str = string(unmappeddat(i));
   A = cat(1, A, strsplit(str, ' '));
end

% plot paths
num_paths = 20;
num_pops = size(A,1) / num_paths;

figure
F(num_paths) = struct('cdata',[],'colormap',[]);
pl_colors = hsv(num_paths);
for j = 1:num_pops
    for k = 1:num_paths
        plot_path(A(k + (j-1)*num_paths,:), pl_colors, k);
    end
    drawnow
    F(j) = getframe(gcf);
end

% repeat as movie
fig = figure;
movie(fig, F, 2);

% helper function to plot path
function pp = plot_path(path, pl_colors, k)
    path_length = 10;
    str = path(1);
    str = strsplit(str, {',', '(', ')'});
    x = str(2);
    y = str(3);
    for i = 2:path_length
        str = path(i);
        str = strsplit(str, {',', '(', ')'});
        x = [x; str(2)];
        y = [y; str(3)];
    end
    X = str2num(char(x));
    Y = str2num(char(y));
    if k == 1
        line([10, 14, 14, 10, 10], [10, 10, 14, 14, 10], 'LineWidth', 2);
        hold off

    else
        hold on
    end

    plot(X,Y,'color',pl_colors(k,:));

end