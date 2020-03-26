function phi = transformPhi(phi, goOver)
    if phi > 360.0
        while phi > 360.0
            phi = phi - 360.0;
        end
    else
        if goOver
            while phi < 20.0
                phi = phi + 360.0;
            end
        else
            while phi < 0.0
                phi = phi + 360.0;
            end
        end
    end
end
