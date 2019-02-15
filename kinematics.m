function x = kinematics(h,p1,p2,startAngle)
c = getDistance(p1,p2);
angle = getAngle(p1, p2, startAngle);
impulses = getImpulsesRotation(angle);
arcLength = getArcLength(angle, 26.5);
if(arcLength ~= 0) 
    rotate(h, impulses, arcLength);
end

speed = getSpeed(0.8, c);
impulsesForward = getImpulsesForward(c);
moveForward(h, impulsesForward, speed);
end

function c = getDistance(p1,p2)
a = abs(p1(1)-p2(1)); % a side of the triangle
b = abs(p1(2)-p2(2)); % b side of the triangle
c = sqrt(power(a,2) + power(b,2));
end

function res = getAngle(p1,p2, startAngle)
a = p2(1)-p1(1); % a side of the triangle
b = p2(2)-p1(2); % b side of the triangle
angleTemp = atan2(b,a); % angle between current position and target center
res=angleTemp-startAngle; 
end

function res = getImpulsesRotation(angle)
radius = 26.5; % radius of vehicle cirle
arcLengthmm = getArcLength(angle, radius);
res = 7.69*arcLengthmm; %7.69imp = 1mm 
end

function res = getArcLength(angle, radius)
res = angle*radius; %mm
end

function res = rotate(h, impulses, distance)
speed = getSpeed(0.2, distance);
kSetSpeed(h,-speed,speed);
    while (1)
    passedImpulses = kGetEncoders(h);
        if (abs(passedImpulses(1)) > abs(impulses))
            kStop(h);
            kSetEncoders(h, 0, 0);
            break;
        end
    res = 0;
    end
end

function res = getSpeed(time, distance)
res = distance/time;
end

function res = moveForward(h, distance, speed)
kSetSpeed(h, speed, speed)
    while(1)
        passedImpulses = kGetEncoders(h);
        abs(passedImpulses(1));
        if(abs(passedImpulses(1)) > distance)
            kStop(h);
            fprintf('STOP');
            break;
        end
    end
    res=0;
end

function res = getImpulsesForward(distancemm)
res = distancemm*7.69;
end

