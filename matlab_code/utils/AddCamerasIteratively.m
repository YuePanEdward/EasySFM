function [tracks,worldPoints,reconstructedPoints,newimg,P,R_t,colorWP]=AddCamerasIteratively(tracks,worldPoints,reconstructedPoints,img,imgdone,P,R_t)

global K;

%%%%%%%%%%%%% STEP 7: GET OTHER IMAGE WHICH MATCHES %%%%%%%%%%%%%

t=sum(tracks(reconstructedPoints,:)>0,1);
t(imgdone)=0;
[~,newimg]=max(t);

%%%%%%%%%%%%% STEP 8: 3 POINT PnP, FIND ESSENTIAL %%%%%%%%%%%%%

idxs=tracks(reconstructedPoints,newimg)>0; %tracks which are in 3D and in the new image
WP=worldPoints(reconstructedPoints(idxs),:);
features=img(newimg).vpts(tracks(reconstructedPoints(idxs),newimg)).Location;

% Try a bunch of

[finalRt, inliers]=ransacP3P(WP,features);
nn = reconstructedPoints(idxs); tracks(setdiff(nn,nn(inliers)),newimg) = 0; % HACK !!!
P{newimg} = K * finalRt;
R_t{newimg} = finalRt;


%%%%%%%%%%%%% STEP 9: TRIANGULATE, AUGMENT 3D MODEL %%%%%%%%%%%%%

% Check for new points which might be added to the model

for jj=imgdone
	newfeat=find(tracks(:,jj) .* tracks(:,newimg) >0 & worldPoints(:,1)==0 & worldPoints(:,2)==0 & worldPoints(:,3)==0);
    if (~isempty(newfeat))
        matchedPoints1=img(jj).vpts(tracks(newfeat,jj));
        matchedPoints2=img(newimg).vpts(tracks(newfeat,newimg));

                
        [WP, RE] = triangulate(matchedPoints1,matchedPoints2,P{jj}',P{newimg}');
        
        C1=-R_t{jj}(1:3,1:3)'*R_t{jj}(1:3,4);
        viewDir1=R_t{jj}(3,1:3)';
        
        C2=-R_t{newimg}(1:3,1:3)'*R_t{newimg}(1:3,4);
        viewDir2=R_t{newimg}(3,1:3)';
        
        % Verification that line through camera center and line through
        % point are in the same hemisphere
        
        newinfront = ((WP-repmat(C1',size(WP,1),1))*viewDir1 > 0) & ((WP-repmat(C2',size(WP,1),1))*viewDir2 > 0);
        repErrOk = (RE<5) & newinfront;
        
        worldPoints(newfeat(repErrOk),1:3)=WP(repErrOk,:); %link worldpoints to tracks
        
        reconstructedPoints=[reconstructedPoints;newfeat(repErrOk)];
        tracks(newfeat(repErrOk),setdiff(imgdone,jj)) = 0; % HACK !!!
        
        %store color info
        numPixels = size(img(jj).img, 1) * size(img(jj).img, 2);
        allColors = reshape(img(jj).img,[numPixels 3]);
        colorIdx = sub2ind([size(img(jj).img, 1), size(img(jj).img, 2)], uint16(matchedPoints1.Location(repErrOk,2)), ...
            uint16(matchedPoints1.Location(repErrOk, 1)));
        colorWP(newfeat(repErrOk),1:3) = allColors(colorIdx, :);
    end
end


% How much error do I have?
disp(['Reprojection error ',num2str(median(RE(repErrOk)))]);

end