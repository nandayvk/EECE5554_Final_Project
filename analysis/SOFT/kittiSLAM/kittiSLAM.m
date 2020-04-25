ImageSize=[375,1242];

IntrinsicMatrix0 = [7.215377e+02 0.000000e+00 6.095593e+02; 0.000000e+00 7.215377e+02 1.728540e+02; 0.000000e+00 0.000000e+00 1.000000e+00];
IntrinsicMatrix1 = [7.215377e+02 0.000000e+00 6.095593e+02; 0.000000e+00 7.215377e+02 1.728540e+02; 0.000000e+00 0.000000e+00 1.000000e+00];

rotationMat01=[9.998817e-01 1.511453e-02 -2.841595e-03; -1.511724e-02 9.998853e-01 -9.338510e-04; 2.827154e-03 9.766976e-04 9.999955e-01];
rotationMat02 = [9.998321e-01 -7.193136e-03 1.685599e-02; 7.232804e-03 9.999712e-01 -2.293585e-03; -1.683901e-02 2.415116e-03 9.998553e-01];

cameraParams0 = cameraParameters('IntrinsicMatrix',IntrinsicMatrix0','ImageSize',ImageSize);
cameraParams1 = cameraParameters('IntrinsicMatrix',IntrinsicMatrix1','ImageSize',ImageSize);

Tvec0 = [4.485728e+01 2.163791e-01 2.745884e-03];
Tvec1 = [-3.395242e+02 2.199936e+00 2.729905e-03];

stereoParams = stereoParameters(cameraParams0,cameraParams1,rotationMat01\rotationMat02,Tvec1-Tvec0);

Scene1 = imageDatastore('\..\data\2011_09_26\2011_09_26_drive_0009_sync\image_02\data\');
Scene2 = imageDatastore('\..\data\2011_09_26\2011_09_26_drive_0009_sync\image_03\data\');
%%
 p = [0; 0; 0 ;1];
for n = 1:size(Scene1.Files)-1
    img_l = readimage(Scene1,n);
    img_l1 = readimage(Scene1,n+1);
    img_r = readimage(Scene1,n);
    img_r1 = readimage(Scene1,n+1);
    i1 = uint8(0.2989 * img_l(:,:,1) + 0.5870 * img_l(:,:,2) + ...
                0.1140 * img_l(:,:,3));
    i2 = uint8(0.2989 * img_r(:,:,1) + 0.5870 * img_r(:,:,2) + ...
                    0.1140 * img_r(:,:,3));
    points1 = detectSURFFeatures(i1, 'MetricThreshold', 2000);
    points2 = detectSURFFeatures(i2, 'MetricThreshold', 2000);
    [f1, vpts1] = extractFeatures(i1, points1);
    [f2, vpts2] = extractFeatures(i2, points2);
    indexPairs = matchFeatures(f1, f2) ;
    matchedPoints1 = vpts1(indexPairs(1:50, 1));
    matchedPoints2 = vpts2(indexPairs(1:50, 2));
    point3d = triangulate(matchedPoints1, matchedPoints2, stereoParams)/1000;
    
    tracker = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(tracker, matchedPoints1.Location, img_l);
    [matchedPoints01, validity] = step(tracker, img_l1);
    tracker = vision.PointTracker('MaxBidirectionalError', 1);
    initialize(tracker, abs(matchedPoints01), img_l1);
    [matchedPoints11, validity] = step(tracker, img_r1);
    point3d1 = triangulate(matchedPoints01, abs(matchedPoints11), stereoParams)/1000;
    
    count = size(point3d,1)+1;
    for i = 1 : size(point3d,1)
        point3d(count-i,3)=abs(point3d(count-i,3));
        point3d1(count-i,3)=abs(point3d1(count-i,3));
        if abs(point3d(count-i,3)-point3d1(count-i,3)) > 3e4
            point3d(count-i,:)=[];
            point3d1(count-i,:)=[];
        end
    end
    pc = pointCloud(point3d);
    pc1 = pointCloud(point3d1);
    tform = pcregrigid(pc1,pc);
   
    p = tform.T'*p;
    scatter(-p(1),-p(3));hold on
end