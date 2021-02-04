function cam_handle = DrawCam(focalL, ImgH, ImgW, T, L, Color, LineWidth)
O=[0; 0; 0];

Depth = L;
Height = L * ImgH / (2 * focalL);
Width = L * ImgW / (2 * focalL);

P1 = [Depth; -Width; -Height];
P2 = [Depth; -Width; Height];
P3 = [Depth; Width;  Height];
P4 = [Depth; Width;  -Height];

O = transform_point(T, O);
P1 = transform_point(T, P1);
P2 = transform_point(T, P2);
P3 = transform_point(T, P3);
P4 = transform_point(T, P4);

A = [O P1 P2 O P2 P3 O P3 P4 O P4 P1];
X = A(1,:);
Y = A(2,:);
Z = A(3,:);

% X = [Origin(1), O(1), O(1), O(1), P1(1), P2(1), P4(1), P3(1);...
%      P1(1),     P2(1),     P3(1),     P4(1),     P2(1), P4(1), P3(1), P1(1)]; 
% Y = [O(2), O(2), O(2), O(2), P1(2), P2(2), P4(2), P3(2);...
%      P1(2),     P2(2),     P3(2),     P4(2),     P2(2), P4(2), P3(2), P1(2)]; 
% Z = [O(3), O(3), O(3), O(3), P1(3), P2(3), P4(3), P3(3);...
%      P1(3),     P2(3),     P3(3),     P4(3),     P2(3), P4(3), P3(3), P1(3)]; 

cam_handle = plot3(X, Y, Z, Color, 'LineWidth', LineWidth); hold on;

% 
% plot3([Origin(1); P1(1)], [Origin(2); P1(2)], [Origin(3); P1(3)], Color, 'LineWidth', LineWidth); hold on;
% plot3([Origin(1); P2(1)], [Origin(2); P2(2)], [Origin(3); P2(3)], Color, 'LineWidth', LineWidth); hold on;
% plot3([Origin(1); P3(1)], [Origin(2); P3(2)], [Origin(3); P3(3)], Color, 'LineWidth', LineWidth); hold on;
% plot3([Origin(1); P4(1)], [Origin(2); P4(2)], [Origin(3); P4(3)], Color, 'LineWidth', LineWidth); hold on;
% plot3([P1(1); P2(1); P4(1); P3(1); P1(1)],...
%       [P1(2); P2(2); P4(2); P3(2); P1(2)],...
%       [P1(3); P2(3); P4(3); P3(3); P1(3)], Color, 'LineWidth', LineWidth);
end

function pt = transform_point(T, p)
    temp = [p;ones(1, size(p,2))];
    temp2 = T*temp;
    pt = temp2(1:3,:);
end
