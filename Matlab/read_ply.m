function read_ply()
ptCloud = pcread('test_mesh.ply');
coordinates=ptCloud.Location; 
x=double(coordinates(:,1)); 
y=double(coordinates(:,2)); 
z=double(coordinates(:,3));
%Generate a delaunay triangulation from 3d vertex coordinates
DT = delaunayTriangulation(x,y,z);
[T,Xb] = freeBoundary(DT); TR = triangulation(T,Xb);
P = incenter(TR); F = faceNormal(TR);
trisurf(T,Xb(:,1),Xb(:,2),Xb(:,3),'FaceColor','cyan','faceAlpha',0.8); 

end