% plotAdaptiveVsLF (adaptive_folder, lf_folder)
%
% Author: Stephanie Kemna, USC, 2014/2015
%
function [] = plotAdaptiveVsLF (adaptive_folder, lf_folder)

if nargin < 1
    disp('ERROR: no arguments received');
    disp('Usage: plotAdaptiveVsLF (adaptive_folder, lf_folder)');
    return
end

vehicles=['A','F','G'];

%% prepare figure
figure('Position',[100,100,1400,1000]);
hold on; grid on;
% lake outline
lake_outline = [900,445;980,450;1115,455;1130,395;1110,360;1115,325;1145,290;1170,300;1310,340;1445,340;1515,300;1470,205;1320,170;1220,95;1105,90;1060,150;1110,265;1080,300;1035,295;990,275;940,255;900,205;900,445];
plot(lake_outline(:,1),lake_outline(:,2),'Color',[0.6 0.3 0],'LineWidth',2);

%% get all data for Adaptive
disp('Adaptive');
for idx=1:length(vehicles),
   nav_x = load([adaptive_folder '/txtFiles/navx_' vehicles(idx) '.txt']);
   nav_xt = load([adaptive_folder '/txtFiles/navxt_' vehicles(idx) '.txt']);
   nav_y = load([adaptive_folder '/txtFiles/navy_' vehicles(idx) '.txt']);
   nav_yt = load([adaptive_folder '/txtFiles/navyt_' vehicles(idx) '.txt']);
   wpts_x = load([adaptive_folder '/txtFiles/wpts_' vehicles(idx) '_x.txt']);
   wpts_y = load([adaptive_folder '/txtFiles/wpts_' vehicles(idx) '_y.txt']);
   if ( vehicles(idx) == 'A' )
       plot(nav_x,nav_y,'-','Color',[0 0 0], 'LineWidth', 2);
   elseif ( vehicles(idx) == 'F' )
       plot(nav_x,nav_y,'--','Color',[0 1 0], 'LineWidth', 2);
       in_poly_ferdinand = inpolygon( nav_x, nav_y, lake_outline(:,1), lake_outline(:,2) );
       ferdinand_inside = sum(in_poly_ferdinand);
       ferdinand_outside = sum(~in_poly_ferdinand);
       disp(['ferdinand percentage outside ' num2str(ferdinand_outside/(ferdinand_inside+ferdinand_outside)) ]);
   elseif ( vehicles(idx) == 'G' )
       plot(nav_x,nav_y,'--','Color',[0.5 0 0.5], 'LineWidth', 2);
       in_poly_gerard = inpolygon( nav_x, nav_y, lake_outline(:,1), lake_outline(:,2) );
       gerard_inside = sum(in_poly_gerard);
       gerard_outside = sum(~in_poly_gerard);
       disp(['gerard percentage outside ' num2str(gerard_outside/(gerard_inside+gerard_outside)) ]);
   end
   %scatter(wpts_x,wpts_y,'.','MarkerFaceColor',[0.5 0 0]);
end

%% finish figure
axis equal;
xlabel('x (m)');
ylabel('y (m)');
title('Adaptive');
set(gca,'FontSize',16);
set(findall(gcf,'type','text'),'FontSize',16)
legend('Lake Outline','ASV','AUV1','AUV2');

%% prepare figure
figure('Position',[100,100,1400,1000]);
hold on; grid on;
% lake outline
lake_outline = [900,445;980,450;1115,455;1130,395;1110,360;1115,325;1145,290;1170,300;1310,340;1445,340;1515,300;1470,205;1320,170;1220,95;1105,90;1060,150;1110,265;1080,300;1035,295;990,275;940,255;900,205;900,445];
plot(lake_outline(:,1),lake_outline(:,2),'Color',[0.6 0.3 0],'LineWidth',2);

%% get all data for LF
disp('Leader-Follower');
for idx=1:length(vehicles),
   nav_x = load([lf_folder '/txtFiles/navx_' vehicles(idx) '.txt']);
   nav_xt = load([lf_folder '/txtFiles/navxt_' vehicles(idx) '.txt']);
   nav_y = load([lf_folder '/txtFiles/navy_' vehicles(idx) '.txt']);
   nav_yt = load([lf_folder '/txtFiles/navyt_' vehicles(idx) '.txt']);
   wpts_x = load([lf_folder '/txtFiles/lf_wpts_' vehicles(idx) '_x.txt']);
   wpts_y = load([lf_folder '/txtFiles/lf_wpts_' vehicles(idx) '_y.txt']);
   if ( vehicles(idx) == 'A' )
       plot(nav_x,nav_y,'-','Color',[0 0 0], 'LineWidth', 2);
   elseif ( vehicles(idx) == 'F' )
       plot(nav_x,nav_y,'--','Color',[0 1 0], 'LineWidth', 2);
       in_poly_ferdinand = inpolygon( nav_x, nav_y, lake_outline(:,1), lake_outline(:,2) );
       ferdinand_inside = sum(in_poly_ferdinand);
       ferdinand_outside = sum(~in_poly_ferdinand);
       disp(['ferdinand percentage outside ' num2str(ferdinand_outside/(ferdinand_inside+ferdinand_outside)) ]);
   elseif ( vehicles(idx) == 'G' )
       plot(nav_x,nav_y,'--','Color',[0.5 0 0.5], 'LineWidth', 2);
       in_poly_gerard = inpolygon( nav_x, nav_y, lake_outline(:,1), lake_outline(:,2) );
       gerard_inside = sum(in_poly_gerard);
       gerard_outside = sum(~in_poly_gerard);
       disp(['gerard percentage outside ' num2str(gerard_outside/(gerard_inside+gerard_outside)) ]);
   end
   %scatter(wpts_x,wpts_y,'.','MarkerFaceColor',[0 0 0.5]);
end

%% finish figure
axis equal;
xlabel('x (m)');
ylabel('y (m)');
title('Leader-Follower');
set(gca,'FontSize',16);
set(findall(gcf,'type','text'),'FontSize',16)
legend('Lake Outline','ASV','AUV1','AUV2');

end