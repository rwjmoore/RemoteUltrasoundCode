[x,y,z] = sphere;
fvc = surf2patch(x,y,z);
X = patch('Faces', fvc.faces, 'Vertices', fvc.vertices, 'FaceColor', [1, 0, 0])
view(3)
plot(X)