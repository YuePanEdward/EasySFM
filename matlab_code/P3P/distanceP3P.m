function [globalinliers,finalRt]=distanceP3P(Rt,points,th)

global K;

globalinliers=0;

for ii=1:size(Rt,3)
    
    chosenRt=Rt(:,:,ii);

    P = K * chosenRt;

    worldpoints=points(1:3,:);
    features=points(4:6,:);
    
    % Write the code that computes the projection error between world
    % points and features given the parameters K, P, chosen Rt and choose
    % the Rt that has more inliers (more points with error less than th)

    

end

% Return inliers and the chosen Rt in finalRt

end