function ReadData()
clc; clear all; close all;
pts = load('points.txt');
poses = load('pose.txt');
img_idx = load('img_idx.txt');

T = zeros(4,4,size(poses,1));
locations = zeros(3,size(poses,1));
for i = 1:size(poses,1)
    T(:,:,i) = reshape(poses(i,:), 4, 4);
    locations(:,i) = T(1:3,4,i);
end

figure(1);
scatter3(pts(:,1), pts(:,2), pts(:,3), 5, pts(:,7:9)/255, 'filled'); hold on;
h1 = plot3(locations(1,1),locations(2,1),locations(3,1),'b'); 
h = DrawCam(526, 344, 472, T(:,:,1), 0.2, 'r', 1.5);
for i=1:size(poses,1)
    delete(h1);
    delete(h);
    h1 = plot3(locations(1,1:i),locations(2,1:i),locations(3,1:i),'b'); 
    h = DrawCam(526, 344, 472, T(:,:,i), 0.2, 'r', 1.5);
    axis equal;
    set(gca,'YDir','reverse');
    set(gca,'ZDir','reverse');
    xlabel X[m];
    ylabel Y[m];
    zlabel Z[m];
    drawnow;
end

surfels = load('surfel.txt');
figure(2);
plot3(0,0,0); hold on;
draw_circle_all(surfels(:,4:6)', surfels(:,1:3)', 0.1, 'r');

Pt1 = surfels(:,4:6)';
Pt2 = surfels(:,4:6)' + surfels(:,1:3)'*0.2;
plot3([Pt1(1,:); Pt2(1,:)], [Pt1(2,:); Pt2(2,:)], [Pt1(3,:); Pt2(3,:)]);

hold off;
axis equal;
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
xlabel X[m];
ylabel Y[m];
zlabel Z[m];






% ptCloud = pointCloud(surfels(:,4:6), 'Normal', surfels(:,1:3));
% pcwrite(ptCloud, 'Surfel', 'PLYFormat', 'binary');
% 
% ptCloud2 = pointCloud(pts(:,1:3), 'Normal', pts(:,4:6), 'Color', pts(:,7:9)/255.0);
% pcwrite(ptCloud2, 'PtCloud', 'PLYFormat', 'binary');


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