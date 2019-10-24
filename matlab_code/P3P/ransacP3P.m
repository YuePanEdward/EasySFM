% tracks 
% 3D world points corresponding to tracks
% new image ID you want to evaluate

function [Rt, inliers]=ransacP3P(WP,features)

global K;

% Prepare points for 5 point algorithm (also bundler)

pts1=inv(K)*[features,ones(size(WP,1),1)]';
% Normalize
pts1=double(pts1./repmat(sqrt(sum(pts1.^2)),3,1));
points=[WP,pts1'];

[Rt, inliers] = ransac(points', @fittingP3P, @distanceP3P, @dummyP3P, 3, 2, 0, 1000, 10000);


end