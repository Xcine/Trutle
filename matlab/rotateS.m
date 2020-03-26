function rotateS(vel,dir)
    v_robot = rospublisher('/mobile_base/commands/velocity') ;
    velmsg = rosmessage(v_robot);
    velmsg.Angular.Z = dir*vel;
    velmsg.Linear.X = 0;
    send(v_robot,velmsg);  

    imsub = rossubscriber('/camera/rgb/image_raw');
    img = receive(imsub);
    I = readImage(img);
    imshow(I);
    velmsg.Angular.Z = 0.0;
end