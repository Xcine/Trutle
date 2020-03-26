function  walkS(velocity)    

    v_robot = rospublisher('/mobile_base/commands/velocity') ;
    velmsg = rosmessage(v_robot);
    velmsg.Linear.X = velocity;
    velmsg.Angular.Z = 0;
    send(v_robot,velmsg);

    imsub = rossubscriber('/camera/rgb/image_raw');
    img = receive(imsub);
    imshow(readImage(img));
    
    velmsg.Linear.X = 0;

    disp("Walking ended.")
    
end