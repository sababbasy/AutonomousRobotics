function [ output_args ] = odometry()
position = 5;
circle = rectangle('Position',[position,position,20,20]);
circle.EdgeColor = 'none';
circle.FaceColor = 'blue';
axis off
axis equal
xlim([10 270])
ylim([10 400])

    while(1)
    shape = rectangle('Position',[position,position,10,10]);
    shape.FaceColor = 'red';
    shape.EdgeColor = 'none';
    axis off
    position = position+1;
    circle.Position = [position, position, 20,20];

    axis off
    drawnow limitrate
    pause(0.05)
        if (position > 170)
            break;
        end
    end
end

