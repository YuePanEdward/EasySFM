function Rt=fittingP3P(points)

worldpoints=points(1:3,:);
features=points(4:6,:);

Rt=ray2p(worldpoints,features);


end