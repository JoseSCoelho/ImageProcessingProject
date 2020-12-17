function [imageseq1,size] = Inicialize()

name = struct('rgb','','depth',''); %Cria��o da struct para guardar os filenames.

D = pwd; %Diret�rio atual (onde tamb�m est�o as imagens).
R = dir(fullfile(D,'.jpg')); % Encontrar todos os ficheiros jpg (rgb).
Dep = dir(fullfile(D,'.png')); % Encontrar todos os ficheiros png (depth images).
RGB = fullfile(D, '*.jpg');
DEPTH = fullfile(D, '*.png');
jpegFiles = dir(RGB);
if length(jpegFiles) < 1
    RGB = fullfile(D, '*.jpeg');
    jpegFiles = dir(RGB);
end
pngFiles = dir(DEPTH);
size = length(pngFiles); %This length is used a lot
for k = 1:size
  imageseq1(k) = name; %Cria��o do array de structs.
  imageseq1(k).rgb = strcat('midair/', jpegFiles(k).name); %Guardar os file names de rgb.
  imageseq1(k).depth = strcat('midair/', pngFiles(k).name); %Guardar os file names de rgb.
end