close all; clear all;

% import raw data
unmappeddat = importdata('data.txt');
str = string(unmappeddat(1));
A = strsplit(str, ' ');
for i=2:size(unmappeddat)
   str = string(unmappeddat(i));
   A = cat(1, A, strsplit(str, ' '));
end

% plot paths
num_paths = size(A,1);

figure
F(num_paths) = struct('cdata',[],'colormap',[]);
pl_colors = hsv(num_paths);
for j = 1:num_paths
    str = A(j,1);
    str = strsplit(str, {',', '(', ')'});
    x = str(2);
    y = str(3);
    for i = 2:20
        str = A(j, i);
        str = strsplit(str, {',', '(', ')'});
        x = [x; str(2)];
        y = [y; str(3)];
    end
    X = str2num(char(x));
    Y = str2num(char(y));
    plot(X,Y,'color',pl_colors(j,:));
    drawnow
    F(j) = getframe(gcf);
end

% repeat as movie
fig = figure;
movie(fig, F, 2);
