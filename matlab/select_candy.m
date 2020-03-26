function position = select_candy()

    position = -1;
    %get picture and find colors
    imsub = rossubscriber('/camera/rgb/image_raw');
    img = receive(imsub);
    I = readImage(img);
    imshow(I);
    grayI = rgb2gray(I);
    R = I(:,:,1);
    R = imsubtract(R,grayI);
    R_b = im2bw(R, 0.3);
    G = I(:,:,2);
    G = imsubtract(G,grayI);
    G_b = im2bw(G, 0.1);
    B = I(:,:,3);
    B = imsubtract(B,grayI);
    B_b = im2bw(B, 0.3);

    %get shapes  480   640
    N = {R_b,B_b};
    max_radii = 0;
    for n=1: length(N)
        [centers, radii, metric] = imfindcircles(N{n}, [10,300]);
        if isempty([centers, radii, metric])
        else
            cur_radii = max(radii);
            id_max = find(radii == max(radii(:)));
            cur_position = centers(id_max);
            if cur_radii > max_radii
                max_radii = cur_radii;
                position = cur_position;
            end
            %viscircles(centers,radii,"EdgeColor","black");
        end

    end
    
end