% Building a environemnt

%only_balls = true;

gazebo = ExampleHelperGazeboCommunicator;
to_kill = ["unit_sphere_1", "unit_sphere_2", "unit_sphere_3", "bookshelf", "table", "cube_20k", "cabinet"];
reset_state = ["ground_plane_0","threedoors","mobile_base"];

% ball1 = ExampleHelperGazeboModel("Ball1");
% spherelink = addLink(ball1,'sphere',.1,'position',[x(n),y(n), 0.7],'color',[0 0 1 1]);
% ball2 = ExampleHelperGazeboModel("Ball2");
% spherelink = addLink(ball2,'sphere',.1,'position',[x(n),y(n), 0.7],'color',[0 0 1 1]);

cur_state = getSpawnedModels(gazebo);

for n = 1: length(cur_state)
    cur_obj = convertCharsToStrings(cur_state{n});
    kill = true;
    for i = 1: length(reset_state)
        if cur_obj == reset_state(i)
            kill = false;
        end
    end
    if ismember(cur_obj,getSpawnedModels(gazebo)) && kill
        removeModel(gazebo,cur_obj);
    end
end
% 
% x = [1.5    3.5    1.5   1.5   2.5     2.5  2.5    3.5    3.5   4.5];
% y = [-1.5  -1    0.5   1.5   -1      0    1     -0.5    0.5   0];
% names = ["Ball1","Ball2","Ball3","Ball4","Ball5","Ball6","Ball7","Ball8","Ball9","Ball10"];
% for n = 1: length(names)
%     if rem(n,2)==0
%         ball = ExampleHelperGazeboModel(names(n));
%         spherelink = addLink(ball,'sphere',.1,'color',[0 0 1 1]);
%         spawnModel(gazebo,ball,[x(n),y(n),0.7]);
%     else
%         ball = ExampleHelperGazeboModel(names(n));
%         spherelink = addLink(ball,'sphere',.1,'color',[1 0 0 0]);
%         spawnModel(gazebo,ball,[x(n),y(n),0.7]);
%     end
% end


x1 = -5.8;
x2 = 6.7;
y1 = -2.15;
y2 = 5.1;

x = x1 + (x2-x1)*rand(1,30);
y = y1 + (y2-y1)*rand(1,30);
names = ["Ball1","Ball2","Ball3","Ball4","Ball5","Ball6","Ball7","Ball8","Ball9","Ball10","Ball11","Ball12","Ball13","Ball14","Ball15","Ball16","Ball17","Ball18","Ball19","Ball20"];
for n = 1: length(names)
    if rem(n,2)==0
        ball = ExampleHelperGazeboModel(names(n));
        spherelink = addLink(ball,'sphere',.1,'color',[0 0 1 1]);
        spawnModel(gazebo,ball,[x(n),y(n),0.7]);
    else
        ball = ExampleHelperGazeboModel(names(n));
        spherelink = addLink(ball,'sphere',.1,'color',[1 0 0 0]);
        spawnModel(gazebo,ball,[x(n),y(n),0.7]);
    end
end

% x1 = -5.8;
% x2 = 6.7;
% y1 = -2.15;
% y2 = 5.1;
% 
% x = x1 + (x2-x1)*rand(1,30);
% y = y1 + (y2-y1)*rand(1,30);
% names = ["Ball1","Ball2","Ball3","Ball4","Ball5","Ball6","Ball7","Ball8","Ball9","Ball10", "Ball11","Ball12","Ball13","Ball14","Ball15","Ball16","Ball17","Ball18","Ball19","Ball20","Ball21","Ball22","Ball23","Ball24","Ball25","Ball26","Ball27","Ball28","Ball29","Ball30"];
% for n = 1: length(names)
%     if rem(n,2)==0
%         ball = ExampleHelperGazeboModel(names(n));
%         spherelink = addLink(ball,'sphere',.1,'color',[0 0 1 1]);
%         spawnModel(gazebo,ball,[x(n),y(n),0.7]);
%     else
%         ball = ExampleHelperGazeboModel(names(n));
%         spherelink = addLink(ball,'sphere',.1,'color',[1 0 0 0]);
%         spawnModel(gazebo,ball,[x(n),y(n),0.7]);
%     end
% end

% for i = 1:25   
%     if rem(i,2)==0
%         %spawnModel(gazebo,ball,[x(i),y(i),0.7]);
%         spawnModel(gazebo,ball,[-3 + (3+3)*rand(1,1),-3 + (3+3)*rand(1,1),0.7]);
%     else
%         %spawnModel(gazebo,ball2,[x(i),y(i),0.7]);
%         spawnModel(gazebo,ball2,[-3 + (3+3)*rand(1,1),-3 + (3+3)*rand(1,1),0.7]);
%     end
%     %pause (1) 
% end


