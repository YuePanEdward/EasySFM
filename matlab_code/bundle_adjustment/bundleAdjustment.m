function [ R_t, P, trackstoWP , reconstructedPoints ] = bundleAdjustment( imgdone, R_t, P, imgs, trackstoWP, tracks, reconstructedPoints)
%RS:
% RS.X
%  RS.X.pts      - 3d points
%  RS.X.cammask  - cell array per point
%   camidx       - camera index
%   uidx         - index of u (image pt)
% RS.cam         - cell array per camera
%  P             - [Rt] matrix (no K)
%  u             - normalized pixel coordinates of image features
%  Xmask         - cell array per u (not needed?)
%  desc          - descriptor per feature (not needed?)
% RS.cams        - indices of cameras (sorted)

% fill RS.cam
global K;

for ii = 1:length(imgdone)
    img_ii = imgdone(ii);
    
    u=inv(K)*[imgs(img_ii).vpts.Location,ones(size(imgs(img_ii).vpts,1),1)]';
    
    % Normalize
    u=double(u./repmat(sqrt(sum(u.^2)),3,1));    
    
    RSin.cam{ii} = struct('P',R_t(img_ii),'u',u);
end
RSin.cams = 1:length(imgdone);
%fill RS.X
xidx = 0;
for jj = reconstructedPoints'

    xidx = xidx+1;
    X(xidx,:) = trackstoWP(jj,:);
    cellidx = 0;
    camfeatpp = {};
    for ii = 1:length(imgdone)
        camidx = ii; % imgdone(ii);
        uidx = tracks(jj,imgdone(ii));
        if uidx ~=0
            cellidx = cellidx+1;
            camfeatpp{cellidx} = struct('camidx',camidx,'uidx',uidx);
        end
    end
    cammask{xidx} = camfeatpp;
    %cammask{ii) = struct
end
RSin.X = struct('pts',{double(X')},'cammask',{cammask});

opt = struct('uvbac',struct('verbose',true));

RSout = call_uvbac(RSin,opt);

% update Rt
for ii = 1:length(imgdone)
    img_ii = imgdone(ii);
    R_t{img_ii} = RSout.cam{ii}.P;
    P{img_ii} = K * R_t{img_ii};
end
%update trackstoWP and X
X = RSout.X.pts';

centerOfMass = [mean(X(:,1)) mean(X(:,2)) mean(X(:,3))];
distanceToCenter = pdist2(X,centerOfMass,'euclidean');
thresh = quantile(distanceToCenter,0.95)*2;

xidx = 0;
eraseP=[];
for jj = reconstructedPoints'

    xidx = xidx+1;
    if distanceToCenter(xidx) > thresh
        trackstoWP(jj,:) = [0,0,0];
        eraseP(end+1)=jj;
        %tracks(jj,imgdone) = zeros(size(imgdone));
    else
        trackstoWP(jj,:) = X(xidx,:);
    end
end

for kk=1:numel(eraseP)
reconstructedPoints(find(reconstructedPoints==eraseP(kk)))=[];
end

X = X(distanceToCenter<=thresh,:);
end

