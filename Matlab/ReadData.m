function ReadData()
pts = load('points.txt');
% 
% figure(1);
% scatter3(pts(:,1), pts(:,2), pts(:,3), 5, pts(:,7:9)/255, 'filled');
% axis equal;
% set(gca,'YDir','reverse');
% set(gca,'ZDir','reverse');
% xlabel X[m];
% ylabel Y[m];
% zlabel Z[m];
% 
surfels = load('surfel.txt');
% figure(2);
% plot3(0,0,0); hold on;
% draw_circle_all(surfels(:,4:6)', surfels(:,1:3)', 0.3, 'r');
% hold off;
% axis equal;
% set(gca,'YDir','reverse');
% set(gca,'ZDir','reverse');
% xlabel X[m];
% ylabel Y[m];
% zlabel Z[m];

ptCloud = pointCloud(surfels(:,4:6), 'Normal', surfels(:,1:3));
pcwrite(ptCloud, 'Surfel', 'PLYFormat', 'binary');

ptCloud2 = pointCloud(pts(:,1:3), 'Normal', pts(:,4:6), 'Color', pts(:,7:9)/255.0);
pcwrite(ptCloud2, 'PtCloud', 'PLYFormat', 'binary');


end

function draw_circle_all(surfel_p, surfel_n, surfel_r, surfel_c)
% surfel_p : Surfel center position
% surfel_n : Surfel normal direction
% surfel_r : Surfel radius
% surfel_c : Surfel color (if needed)

t = 0:0.1:2*pi;
z = zeros(size(t,2), 1);
x = surfel_r*cos(t');
y = surfel_r*sin(t');

for i=1:size(surfel_n,2)
    [R, ~] = calculate_R([0;0;1], surfel_n(:,i));
    T = [R surfel_p(:,i);0 0 0 1];
    X = [x';y';z'; ones(1, size(x,1))];
    X2 = T*X;
    X2 = X2(1:3,:);
    X2 = [X2 X2(:,1)];

    fill3(X2(1,:), X2(2,:), X2(3,:), surfel_c);
end
end


function [R, q] = calculate_R(v1, v2)
% v1 : Base
% v2 : Target

v3 = cross(v1,v2);
v3 = v3/norm(v3);
theta = acos((v1'*v2)/(norm(v1)*norm(v2)));

etha = cos(theta/2);
epsilon = v3*sin(theta/2);

q = [etha; epsilon];

R = quat2rotm(q');
end