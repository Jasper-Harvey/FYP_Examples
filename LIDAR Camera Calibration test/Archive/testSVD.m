clear all
close all

load imgCnrs
load lidarCnrs

for i=1:size(imgCnrs,1)-1
%    imgP(:,:,i) = squeeze(imgCnrs(i,:,:)).'
%    lidarP(:,:,i) = squeeze(lidarCnrs(i,:,:)).'
   
   Pimg(((i-1)*4 + 1):((i-1)*4 + 1)+3, :) = squeeze(imgCnrs(i,:,:));
   Plidar(((i-1)*4 + 1):((i-1)*4 + 1)+3, :) = squeeze(lidarCnrs(i,:,:));

end


adjustPimg = Pimg - mean(Pimg);
adjustPlidar = Plidar - mean(Plidar);

W = adjustPlidar.'*adjustPimg;
[U,~,V] = svd(W);

R = (V*diag([ones(1,2) sign(det(U*V'))])*U.') ;% Handle reflection
t = mean(Pimg).' - (R*mean(Plidar).')

tfP = (R*Plidar.').' + t.';

figure(1)
for i=1:4:length(Plidar)
    fill3(Pimg(i:(i+3),1), Pimg(i:(i+3),2), Pimg(i:(i+3),3), 'c');
    hold on
    plot3(Pimg(i,1),Pimg(i,2), Pimg(i,3), '.r', "MarkerSize", 20)
    plot3(Pimg(i+1,1),Pimg(i+1,2), Pimg(i+1,3), '.g', "MarkerSize", 20)
    plot3(Pimg(i+2,1),Pimg(i+2,2), Pimg(i+2,3), '.b', "MarkerSize", 20)
    plot3(Pimg(i+3,1),Pimg(i+3,2), Pimg(i+3,3), '.c', "MarkerSize", 20)
    
    fill3(tfP(i:(i+3),1), tfP(i:(i+3),2), tfP(i:(i+3),3), 'r');
    plot3(tfP(i,1),tfP(i,2), tfP(i,3), '.r', "MarkerSize", 20)
    plot3(tfP(i+1,1),tfP(i+1,2), tfP(i+1,3), '.g', "MarkerSize", 20)
    plot3(tfP(i+2,1),tfP(i+2,2), tfP(i+2,3), '.b', "MarkerSize", 20)
    plot3(tfP(i+3,1),tfP(i+3,2), tfP(i+3,3), '.c', "MarkerSize", 20)

end
daspect([1 1 1])


