function x = basic_mov(h)
turn = 0;
leftTurnSpeed = -70;
rightTurnSpeed = 70;
distance = 750;
kSetSpeed(h, 450, 450);
    while (1)
        if(kGetEncoders(h) > distance)
            if (turn == 2)
                kStop(h);
                return;
            end
        kSetEncoders(h,0,0);
        kSetSpeed(h, leftTurnSpeed, rightTurnSpeed);
            while(1)
                turnRate = kGetEncoders(h);
                if(abs(turnRate(2)) > 340)
                    kSetEncoders(h,0,0);
                    kSetSpeed(h, 450, 450);
                    if (turn == 1)
                    distance = 700;
                    else
                    distance = distance*3 + 105; 
                    end
                    turn = turn +1;
                    leftTurnSpeed = 70;
                    rightTurnSpeed = -70;
                    break;
                end
            end
        end
    end    
end

