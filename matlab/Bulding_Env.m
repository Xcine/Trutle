% Building a environemnt


gazebo = ExampleHelperGazeboCommunicator;
to_kill = ["unit_sphere_1", "unit_sphere_2", "unit_sphere_3", "bookshelf", "table", "cube_20k", "cabinet"];

for n = 1: length(to_kill)
    if ismember(to_kill(n),getSpawnedModels(gazebo))
        removeModel(gazebo,to_kill(n));
    end
end

x = [1.5    3.5    1.5   1.5   2.5     2.5  2.5    3.5    3.5   4.5];
y = [-1.5  -1    0.5   1.5   -1      0    1     -0.5    0.5   0];
names = ["Ball1","Ball2","Ball3","Ball4","Ball5","Ball6","Ball7","Ball8","Ball9","Ball10"];
for n = 1: length(names)
    if rem(n,2)==0
        ball = ExampleHelperGazeboModel(names(n));
        spherelink = addLink(ball,'sphere',.1,'position',[-3,0, 1.2],'color',[0 0 1 1]);
        spawnModel(gazebo,ball,[x(n),y(n),0.7]);
    else
        ball = ExampleHelperGazeboModel(names(n));
        spherelink = addLink(ball,'sphere',.1,'position',[-3,0, 1.2],'color',[1 0 0 0]);
        spawnModel(gazebo,ball,[x(n),y(n),0.7]);
    end
end

% for i = 1:25   
%     if rem(i,2)==0
%         spawnModel(gazebo,ball,[x(i),y(i),0.7]);
%         %spawnModel(gazebo,ball,[-3 + (3+3)*rand(1,1),-3 + (3+3)*rand(1,1),0.7]);
%     else
%         spawnModel(gazebo,ball2,[x(i),y(i),0.7]);
%         %spawnModel(gazebo,ball2,[-3 + (3+3)*rand(1,1),-3 + (3+3)*rand(1,1),0.7]);
%     end
%     %pause (1) 
% end


