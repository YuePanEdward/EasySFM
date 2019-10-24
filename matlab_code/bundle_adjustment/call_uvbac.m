function RS = call_uvbac(RS,opt)

cams = sort(RS.cams);
camidx = zeros(1,max(cams)); for i=1:length(cams), camidx(cams(i)) = i; end;
projcnt = 0; for i=1:size(RS.X.pts,2), projcnt = projcnt + length(RS.X.cammask{i}); end;

k = 0;
proj = zeros(5,projcnt);
for i=1:size(RS.X.pts,2)
  for j=1:length(RS.X.cammask{i})
    k = k+1;
    proj(:,k) = [camidx(RS.X.cammask{i}{j}.camidx); i; RS.cam{RS.X.cammask{i}{j}.camidx}.u(:,RS.X.cammask{i}{j}.uidx)];
  end
end

[~, idx] = unique(proj(1:2,:)','rows');
u = sparse(proj(1,idx),proj(2,idx),proj(3,idx),length(cams),size(RS.X.pts,2));
v = sparse(proj(1,idx),proj(2,idx),proj(4,idx),length(cams),size(RS.X.pts,2));
w = sparse(proj(1,idx),proj(2,idx),proj(5,idx),length(cams),size(RS.X.pts,2));

caat = zeros(6,length(cams));
for i=1:length(cams)
  Rx = logm(RS.cam{cams(i)}.P(1:3,1:3));
  caat(:,i) = [Rx(3,2); Rx(1,3); Rx(2,1); RS.cam{cams(i)}.P(1:3,4)];
end

[caat RS.X.pts] = uvbac(caat,RS.X.pts,u,v,w,opt.uvbac);

for i=1:length(cams)
  R = expm([0 -caat(3,i) caat(2,i); caat(3,i) 0 -caat(1,i); -caat(2,i) caat(1,i) 0]);
  RS.cam{cams(i)}.P = [R caat(4:6,i)];
end
