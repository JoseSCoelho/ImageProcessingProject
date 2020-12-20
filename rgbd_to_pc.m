function [pc_rgb1,pc_rgb2,W_xyz1,W_xyz2] = rgbd_to_pc(imseq,index1,index2,cam)

%Abrir as depth images que me interessam.
% depth1 = imread(imseq(index1).depth);
% depth2 = imread(imseq(index2).depth);

depth1 = load(imseq(index1).depth);
depth1 = depth1.depth_array;

depth2 = load(imseq(index2).depth);
depth2 = depth2.depth_array;

%Retirar os valores de Z da depth image correspondente.
z1 = double(depth1)*power(10,-3); %Converte milimetros em metros.
z2 = double(depth2)*power(10,-3);

%Intrisic Parameters (Depth Camera)
K = cam.Kdepth;

pc_npoints1 = size(z1, 1)*size(z1, 2);
pc_npoints2 = size(z2, 1)*size(z2, 2);
W_xyz1 = zeros(pc_npoints1, 3);
W_xyz2 = zeros(pc_npoints2, 3);
i1 = 1;
i2 = 1;

%Apply Camera Model to get 3D points in the depth camera coordinate system
for v = 1:640
    for u = 1:480
        if z1(u,v) > 0 
            W_xyz1(i1, :) = K\(z1(u,v)*[v; u; 1]); %XYZ without R and T applied
            i1 = i1+1;
        end
        if z2(u,v) > 0 
            W_xyz2(i2, :) = K\(z2(u,v)*[v; u; 1]);
            i2 = i2+1;
        end
    end
    v
end

%Extrinsic Parameters (RGB Camera)
R = cam.R;
T = cam.T;

%Intrinsic Parameters (RGB Camera)
K_rgb = cam.Krgb;

%Get 3D points in the RGB camera coordinate system
rgb_xyz1 = [R T]*[W_xyz1 ones(length(W_xyz1), 1)]'; 
rgb_xyz2 = [R T]*[W_xyz2 ones(length(W_xyz2), 1)]';
%Get corresponding 2D points (in pixels)
omega1 = K_rgb*rgb_xyz1;
omega2 = K_rgb*rgb_xyz2;


omega1 = omega1';
omega2 = omega2';
omega_idx1 = find(omega1(:, 3) > 0);
v1 = ceil(omega1(omega_idx1, 2)./omega1(omega_idx1, 3));
u1 = ceil(omega1(omega_idx1, 1)./omega1(omega_idx1, 3));

omega_idx2 = find(omega2(:, 3) > 0);
v2 = ceil(omega2(omega_idx2, 2)./omega2(omega_idx2, 3));
u2 = ceil(omega2(omega_idx2, 1)./omega2(omega_idx2, 3));

u1(u1(:)>640)=640;
v1(v1(:)>480)=480;
u2(u2(:)>640)=640;
v2(v2(:)>480)=480;

im1 = imread(imseq(index1).rgb);
pc_rgb1 = zeros(length(omega1), 3);
R1 = im1(:, :, 1);
G1 = im1(:, :, 2);
B1 = im1(:, :, 3);

im2 = imread(imseq(index2).rgb);
pc_rgb2 = zeros(length(omega2), 3);
R2 = im2(:, :, 1);
G2 = im2(:, :, 2);
B2 = im2(:, :, 3);


%Get rgb pixel values associated to each 3D point 
for i=1:length(u1)
    if u1(i) > 0
        pc_rgb1(i, :) = [R1(v1(i),u1(i)) G1(v1(i),u1(i)) B1(v1(i),u1(i))];
    end
end

for i=1:length(u2)
    if u2(i) > 0
        pc_rgb2(i, :) = [R2(v2(i),u2(i)) G2(v2(i),u2(i)) B2(v2(i),u2(i))];
    end
end
% 
%figure;
%pcshow(W_xyz2, uint8(pc_rgb2));
% xlabel('X');
% ylabel('Y');
% zlabel('Z');