function ptCloud=PlotPointCloud(R_t,points)

% Show results
figure;
for ii=1:numel(R_t)
    if (~isempty(R_t{ii}))
        Rp = R_t{ii}(:,1:3);
        tp = R_t{ii}(:,4);
        plotCamera('Location',-tp' * Rp, 'Orientation', Rp, 'Size', 0.01,'Label',num2str(ii));
        hold on;
        grid on;
    end
end

ptCloud = pointCloud(points);
%pcwrite(ptCloud,'awsomeModel','PLYFormat','binary');
showPointCloud(ptCloud,'VerticalAxisDir', 'down', 'MarkerSize', 10);


end