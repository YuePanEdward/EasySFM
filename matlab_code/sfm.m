% Pipeline
%
% 1. Extract features (SURF)
% 2. Match features (all images with all images)
% 3. Find the feature tracks which are most popular, and from that the pair
%    of images that contains most of those features
% 4. 5 point algorithm to find Essential matrix
% 5. Find the rotation and translation by checking number of points in
%    front of the camera
% 6. Triangulate and create 3D model
% 7. Bundle adjustment
% 8. Choose the next suitable camera to be matched to the model
% 9. 3-point algorithm PnP, for matching new image to existing 3D model,
%    and finding the new Essential matrices
% 10. Triangulate and augment 3D model with new features
% 11. Run bundle adjustment and repeat 8-9-10 until all cameras are added
%
% Exercise 1: steps 1,2,3
% Exercise 2: step 5
% Exercise 3: step 9

%%%%%%%%%%%%% PRE-PROCESSING %%%%%%%%%%%%%

% Add the appropriate paths
addpath(genpath('.'));

numi=11; %number of images

% get K

filenameK = 'data/k/K.txt';
fileID = fopen(filenameK,'r');
formatSpec = '%f %f %f';
sizeK = [3 3];
global K;
K = fscanf(fileID,formatSpec,sizeK);
K = K';
fclose(fileID);

% load images

for ii=1:numi
    img(ii).img = imread(['data/images/',sprintf('%3.4d',ii-1),'.png']);
    img(ii).imggray = rgb2gray(img(ii).img);
end

%%%%%%%%%%%%% STEP 1: FEATURES %%%%%%%%%%%%%

% Detection of the SURF features
strongest_K=50;

for ii=1:numi
    img(ii).keypoints = detectSURFFeatures(img(ii).imggray);
    figure(ii);
    imshow(img(ii).imggray); hold on;
    plot(img(ii).keypoints.selectStrongest(strongest_K)); hold off;
    [img(ii).f img(ii).vpts]=extractFeatures(img(ii).imggray, img(ii).keypoints);
end



%{

% Extraction of the descriptors

% HINT: Save the features and descriptors inside the variable img
% img(ii).f = feature descriptors
% img(ii).vpts = valid features after extraction
% Check function "extractFeatures" from MatLab


%%%%%%%%%%%%% STEP 2-3: MATCH FEATURES AND FIND MOST POPULAR %%%%%%%%%%%%%


% Match all features from all images





% Find most significant tracks, those which are longer
% HINT: You need to create a structure called "tracks" (matrix), where each ROW
% represents a track, and each column will represent an image.
% If track i is formed by 2D points found on images 1,2,3, the entries
% (i,1), (i,2) and (i,3) will contain the feature id that corresponds to
% that track. All other entries (i,3:number of images)=0.

% See which pair of images (img1,img2) is best for starting (pair of images that
% contains more long tracks)



% BEGIN!

% Start with the first pair of images
% matchedPoints1 are the matched points in the first image of the pair
% (same for matchedPoints2 of the second image)

% Verify visually that the matches features make sense
plot_matches(img(img1).imggray,img(img2).imggray,matchesPoints1,matchedPoints2);




%%%%%%%%%%%%% STEP 4: ESSENTIAL MATRIX WITH 5-POINT ALGORITHM %%%%%%%%%%%%%


th = .002;  % Distance threshold for deciding outliers
[E, inliers] = ransac5point([matchedPoints1,ones(size(matchedPoints1,1),1)]',[matchedPoints2,ones(size(matchedPoints2,1),1)]', th, K, 1);

% Filter matched points and take only those that RANSAC considers inliers
matchedPoints1=matchedPoints1(inliers);
matchedPoints2=matchedPoints2(inliers);



    
%%%%%%%%%%%%%%% STEP 5: FIND ROTATION AND TRANSLATION OF THE CAMERAS %%%%%%%%%%%%%

% Obtain R,t by using Singular Value Decomposition of E



% With this, you get 4 possible solutions of R,t
% Test which of the solutions produces MORE points IN FRONT of the cameras
% Consider R of the first image to be the identity matrix, and the
% translation to be a zero vector
% Filter out the points which are NOT in front



% Compute P and R_t for all images
P{img1}= K * [eye(3:3),zeros(3,1)];
P{img2}= K * finalRt;
R_t{img1} = [eye(3:3),zeros(3,1)];
R_t{img2} = double(finalRt);




%%%%%%%%%%%%% STEP 6: TRIANGULATE TO OBTAIN 3D POINTS %%%%%%%%%%%%%

% HINT: check out the "triangulate" function of MatLab
%
% HINT 2: You need to create a structure called "worldPoints" (matrix), where each ROW
% represents a track, and each column will represent a coordinate(x,y,z).
% Each track that gets reconstructed, will get valid 3D world coordinate
% positions on the corresponding row. All other values will be zero.
% Additionally, you will keep a vector called "reconstructedPoints" that
% will contain the indices of the tracks that have been already reconsucted
% (therefore their corrsponding worldPoints row is filled with
% information).




% Plot results before bundler
PlotPointCloud(R_t,worldPoints(reconstructedPoints,:));



% Store the indices of the images that are completed
imgdone=[img1,img2];




%%%%%%%%%%%%% STEP 7: BUNDLE ADJUSTMENT %%%%%%%%%%%%%

[R_t, P, trackstoWP, reconsrtuctedPoints] = bundleAdjustment(imgdone, R_t, P, img, worldPoints, tracks, K,reconsrtuctedPoints);



% Verify the model visually again
PlotPointCloud(R_t,worldPoints);



%close all;

while(numel(imgdone)<numi) %add one image at a time
    
    
    %%%%%%%%%%%%%%%%%%%%%% STEPS 8,9,10  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [tracks,worldPoints,reconstructedPoints,newimg,P,R_t,colorWP]=AddCamerasIteratively(tracks,worldPoints,reconstructedPoints,img,imgdone,P,R_t);
    
    
    %%%%%%%%%%%%% STEP 11: BUNDLE ADJUSTMENT %%%%%%%%%%%%%
    imgdone=[imgdone,newimg];
    
    [R_t, P, worldPoints,reconstructedPoints] = bundleAdjustment(imgdone, R_t, P, img, worldPoints, tracks, reconstructedPoints);
    
    
end


% Plot the final model
ptCloud=PlotPointCloud(R_t,worldPoints(reconstructedPoints,:));

%DONE!!
%}
