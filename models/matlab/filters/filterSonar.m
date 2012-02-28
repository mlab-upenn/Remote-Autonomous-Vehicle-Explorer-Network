clc;
%% PARSE TEXT FILE
dataIn = textread('test.txt', '', 'delimiter', ',', ... 
                'emptyvalue', NaN);
            
%% DEFINE FILTER
alpha = 0.1;
throwOut = 70;

dataOut = [];
for dataSet = 1:length(dataIn(:,1))
    if dataSet > 1
        newDataSet = [];
        for i = 1:length(dataIn(1,:))
            if abs(dataIn(dataSet,i) - dataIn(dataSet-1,i)) > throwOut
                dataIn(dataSet,i) = dataIn(dataSet-1,i);
            end
            newDataPoint = (dataOut(dataSet-1,i) + (alpha)*(dataIn(dataSet,i)-dataOut(dataSet-1,i)));
            newDataSet = [newDataSet newDataPoint];
        end
        dataOut = [dataOut; newDataSet];
    else
        dataOut = [dataOut; dataIn(dataSet,:)];
    end
end
plot(dataIn(:,1),'.-r');
hold on;
plot(dataOut(:,1),'-b');