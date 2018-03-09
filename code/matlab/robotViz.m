function [] = robotViz()

    % DH Parameters for 6 links + tool end
    % World frame is joint 0 with robot base being joint 1
    % this makes 8 links in total
    nlinks = 8;
    a = [0, 0, 0, 0.25, 0.25, 0, 0, 0];
    d = [0, 0.75, 0, 0, 0, 0, 0, 0.05];
    alpha = [pi/2, -pi/2, pi/2, 0, 0, -pi/2, 0, 0];
    theta = [pi/2, 0, 0, 0, 0, 0, 0, 0];
    Rotary = [0, 0, 1, 1, 1, 1, 1, 1];
    com = [0, 0, 0, 0, 0, 0, 0, 0];
    mass = [0, 0, 0, 0, 0, 0, 0, 0];
    inertia = [0, 0, 0, 0, 0, 0, 0, 0];
    
    % Generate set of links
    for i = 1:nlinks
        if (Rotary(i))
            linkList(i) = createLink(a(i), d(i), alpha(i), [], com(i), ...
                mass(i), inertia(i));
        else
            linkList(i) = createLink(a(i), [], alpha(i), theta(i), ...
                com(i), mass(i), inertia(i));
        end
    end
    
    % Draw structure
    paramList = zeros(nlinks,1);
    paramList(2) = d(2);
    drawLinks(linkList, paramList);

end