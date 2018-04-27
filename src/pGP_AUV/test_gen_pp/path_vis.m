close all
clear all
unmappeddat = importdata('data.txt');
str = string(unmappeddat(1));
A = strsplit(str, ' ');
for i=2:size(unmappeddat)
   str = string(unmappeddat(i));
   A = cat(1, A, strsplit(str, ' '));
end
figure
F(510) = struct('cdata',[],'colormap',[]);
for j = 1:510
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
%     scatter(X,Y);
    plot(X,Y);
    drawnow
    F(j) = getframe(gcf);
   
end

fig = figure;
movie(fig, F, 2);
