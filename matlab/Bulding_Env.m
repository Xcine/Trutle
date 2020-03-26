% Building a environemnt
% Arash

%% adding balls
gazebo = ExampleHelperGazeboCommunicator;
ball = ExampleHelperGazeboModel('Ball')
ball2 = ExampleHelperGazeboModel('Ball2')
spherelink = addLink(ball,'sphere',.1,'position',[-3,0, 1.2],'color',[0 0 1 1])
spherelink2 = addLink(ball2,'sphere',.1,'position',[-5,0, 1.2],'color',[1 0 0 0])

%%Pink color = [1 0 1 1], blue color = [0 0 1 1], black= [0 0 0 1], red = [1 0 0 0]
%spawnModel(gazebo,ball,[0,1,0])
%spawnModel(gazebo,ball2,[0,1,0])

%pause

if ismember('Ball',getSpawnedModels(gazebo))
    removeModel(gazebo,'Ball');
end

%pause

 if ismember('Ball2',getSpawnedModels(gazebo))
    removeModel(gazebo,'Ball2');
end

%spawnModel(gazebo,ball,[8.5,0,1])

%pause
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


