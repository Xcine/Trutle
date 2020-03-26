function rotate2circle(from_all)
    if from_all == true
        phi = select_candy_from_all();
        rotate2phi(phi);
        %max_radii = select_candy_from_all2();
        %max_radii
        %rotate2radii(max_radii);
        
    end
    
    position = select_candy();
 
    while ~((position > 310) && (position < 330))
        if position < 320
            dir = 1.0; %turn left
        elseif position > 320
            dir = -1.0; %turn right
        else
            dir = -1,0;
        end
        if abs(position-320.0) < 20
            rotateS(dir, 0.05);
        elseif abs(position-320.0) < 40
            rotateS(dir, 0.05);
        elseif abs(position-320.0) < 70
            rotateS(dir, 0.05);
        else
            rotateS(dir, 0.3);
        end
        position = select_candy();
        position
    end
    
    line([position,position],[0,480]);
    disp("centered.")
 
end