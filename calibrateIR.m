function offsetVector = calibrateIR(h,n,i)
testResults = runTest(h,n);
meanValues = getMean(testResults,n);
stdValues = getStd(testResults,n);

meanSensor1=meanValues(i)
stdSensor1=stdValues(i)
offsetVector=[meanSensor1; stdSensor1];
end

function res = runTest(h,n)
m=0.0;
m(50,8) = 0.0;

    for i=1:1:n
        sensorReadings = kProximity(h);
        for j=1:1:8
            m(i,j) = sensorReadings(j);
        end
    end
res =m;
end

function sensorStd=getStd(matrix,n)
sensorStd = [0;0;0;0;0;0;0;0];
    for i=1:1:8
        column = matrix(:,i);
        sensorStd(i)=std(column);
    end
end

function sensorMeans = getMean(matrix,n)
temp = 0;
sensorMeans=[0;0;0;0;0;0;0;0];
    for i=1:1:8
        for j=1:1:n
            temp = temp + matrix(j,i);
        end
        sensorMeans(i)=temp/n;
        temp=0;
    end
end
