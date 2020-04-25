%% Read the calibration file to find parameters of the cameras
% !! TO-DO: Read from the calib_file instead

% calibration parameters for sequence 2010_03_09_drive_0000
cam_params.fx = 7.215377e+02;               % focal length (u-coordinate) in pixels
cam_params.cx = 6.095593e+02;               % principal point (u-coordinate) in pixels
cam_params.fy = 7.215377e+02;               % focal length (v-coordinate) in pixels
cam_params.cy = 1.728540e+02;               % principal point (v-coordinate) in pixels
cam_params.base = 3.875744e+02;             % baseline in meters (absolute value)

%% Parameters for Feature Extraction
vo_params.feature.nms_n = 8;                      % non-max-suppression: min. distance between maxima (in pixels)
vo_params.feature.nms_tau = 800;                   % non-max-suppression: interest point peakiness threshold
vo_params.feature.margin = 21;                    % leaving margin for safety while computing features ( >= 25)

%% Parameters for Feature Matching
vo_params.matcher.match_binsize = 50;             % matching bin width/height (affects efficiency only)
vo_params.matcher.match_radius = 200;             % matching radius (du/dv in pixels)
vo_params.matcher.match_disp_tolerance = 1;       % dx tolerance for stereo matches (in pixels)
vo_params.matcher.match_ncc_window = 21;          % window size of the patch for normalized cross-correlation
vo_params.matcher.match_ncc_tolerance = 0.3;      % threshold for normalized cross-correlation
% !! TO-DO: add subpixel-refinement using parabolic fitting
vo_params.matcher.refinement = 2;                 % refinement (0=none,1=pixel,2=subpixel)

%% Paramters for Feature Selection using bucketing
vo_params.bucketing.max_features = 1;             % maximal number of features per bucket
vo_params.bucketing.bucket_width = 50;            % width of bucket
vo_params.bucketing.bucket_height = 50;           % height of bucket
% !! TO-DO: add feature selection based on feature tracking
vo_params.bucketing.age_threshold = 10;           % age threshold while feature selection

%% Paramters for motion estimation
% !! TO-DO: use Nister's algorithm for Rotation estimation (along with SLERP) and
% estimate translation using weighted optimization equation
vo_params.estim.ransac_iters = 200;              % number of RANSAC iterations
vo_params.estim.inlier_threshold = 2.0;          % fundamental matrix inlier threshold
vo_params.estim.reweighing = 1;                  % lower border weights (more robust to calibration errors)
%% 
Scene1 = imageDatastore('..\data\2011_09_26\2011_09_26_drive_0022_sync\image_02\data\');
Scene2 = imageDatastore('..\data\2011_09_26\2011_09_26_drive_0022_sync\image_03\data\');
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
camMatrix0 = [7.215377e+02 0.000000e+00 6.095593e+02 0; 0.000000e+00 7.215377e+02 1.728540e+02 0; 0.000000e+00 0.000000e+00 1.000000e+00 0];
camMatrix1 = [7.215377e+02 0.000000e+00 6.095593e+02 -3.875744e+02; 0.000000e+00 7.215377e+02 1.728540e+02 0; 0.000000e+00 0.000000e+00 1.000000e+00 0];
%%
pos = [0;0;0];
pose = [];
Rpos = eye(3);
start = 0;
T = [1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 1];
%p = [0; 0; 0 ;1];
for n = 1:size(Scene1.Files)-1
    img_l = readimage(Scene1,n);
    img_l1 = readimage(Scene1,n+1);
    img_r = readimage(Scene2,n);
    img_r1 = readimage(Scene2,n+1);
    I1_l = uint8(0.2989 * img_l(:,:,1) + 0.5870 * img_l(:,:,2) + ...
                0.1140 * img_l(:,:,3));
    I1_r = uint8(0.2989 * img_r(:,:,1) + 0.5870 * img_r(:,:,2) + ...
                    0.1140 * img_r(:,:,3));
    I2_l = uint8(0.2989 * img_l1(:,:,1) + 0.5870 * img_l1(:,:,2) + ...
                0.1140 * img_l1(:,:,3));
    I2_r = uint8(0.2989 * img_r1(:,:,1) + 0.5870 * img_r1(:,:,2) + ...
                    0.1140 * img_r1(:,:,3));
    if (start == 0)
        vo_previous.pts1_l = computeFeatures(I2_l, vo_params.feature);
        vo_previous.pts1_r = computeFeatures(I2_r, vo_params.feature);
        start = 1;
        I1_l = I2_l;
        map = pointCloud([0 0 0]);
        I1_r = I2_r;
        fprintf('\n---------------------------------\n');
        continue;
    end
    [R, tr, vo_previous,points3D_1] = visualSOFT(n, I1_l, I2_l, I1_r, I2_r, camMatrix0, camMatrix1, vo_params, vo_previous);
    pos = pos + Rpos * tr'
    Rpos = R * Rpos;
    pose = [pose;pos'];
    pc1 = pointCloud(points3D_1');
    Tn = [Rpos tr'; 0 0 0 1];
    T = Tn* T;
    pcnew = pctransform(pc1,affine3d(T'));
    map = pcmerge(pcnew,map,1);
    
    scatter( - pos(1), pos(3), 'b', 'filled');hold on
end