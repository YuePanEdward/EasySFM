function plot_matches(I1,I2,q1,q2)
% Inputs:
%
% I1 = first image
% I2 = second image
% q1 = matched points on the first image (Nx2)
% q2 = matched interest points on the second image (Nx2)
%

offset = size(I1,2);
figure;
imshow([I1 I2]);
hold on;
for u = 1:size(q1,1)
    plot([q1(u,1) offset+q2(u,1)],[q1(u,2) q2(u,2)],'o-','color',rand(1,3));
end

end