function color = checkColor(r,g,b)
    if ismember(1,r)
        %disp("I see red");
        color = "red";
    elseif ismember(1,g)
        %disp("I see green");
        color = "green";
    elseif ismember(1,b)
        %disp("I see blue");
        color = "blue";
    else
        color = "NaN";
    end
end