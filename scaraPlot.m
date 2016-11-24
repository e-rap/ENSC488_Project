fileID = fopen('points.txt', 'r');
formatSpec = '%f %f %f ';
sizeA = [3 1200];
A = fscanf(fileID, formatSpec, sizeA);
fclose(fileID);

A = A';

[m,n] = size(A);

x = A(1:m, 1);
y = A(1:m, 2);
z = A(1:m, 3);

figure 
plot3(x, y, z)


