function obstacle(h, distance)
sensorsCurrentValues = [0;0;0;0;0;0;0;0];
calibrated=[3500,5;3900,5;3700,5;3900,5;3900,5;4000,5;4000,5;4000,5];
c=bar(sensorsCurrentValues);
    while(1)
    sensorsReadings = kProximity(h);
     sensorsReadings2 = kProximity(h);
      for i=1:1:8
          mn=calibrated(i,:);
          mn
          convertedValue=convertTocm(normalize(sensorsReadings(i),mn));
          sensorsCurrentValues(i)=convertedValue;
      end
      delete(c)
      c=bar(sensorsCurrentValues);
      pause(0.02)
      drawnow limitrate
    normalizedValue=normalize((sensorsReadings(1)+sensorsReadings(2))/2,calibrated(8,:));
    convertedValue = convertTocm(normalizedValue);
        if(convertedValue<distance+0.3)
            kStop(h);
            break;
        end
    end
end

function normalizedValue = normalize(value,range)
normalizedValue=(value-range(2))/(range(1)-range(2));

end

function result = convertTocm(normalizedValue)
result=-1.3*log(normalizedValue)-1.3;
end
