clc;
%% PARSE TEXT FILE
dataIn = textread('real_flight_outside.csv', '', 'delimiter', ',', ... 
                'emptyvalue', NaN);
            
%% DEFINE FILTER
alpha = 0.5;
throwOut = 100000;

dataOut = [];
for dataSet = 1:length(dataIn(:,1))
    if dataSet > 1
        newDataSet = [];
        for i = 1:length(dataIn(1,:))
            if abs(dataIn(dataSet,i) - dataIn(dataSet-1,i)) > throwOut
                dataIn(dataSet,i) = dataIn(dataSet-1,i);
            end
            newDataPoint = (1-alpha)*dataOut(dataSet-1,i) + (alpha)*(dataIn(dataSet,i));
            newDataSet = [newDataSet newDataPoint];
        end
        dataOut = [dataOut; newDataSet];
    else
        dataOut = [dataOut; dataIn(dataSet,:)];
    end
end
%subplot(2,1,1);
plot(dataIn(:,1),'.-r');
hold on;
plot(dataIn(:,2),'.b');
plot(dataIn(:,3),'-g');
%plot(dataOut(:,1),'-b','LineWidth',2);
xlabel('Sample Number (20 Hz)');
ylabel('Amplitude (dps)');
title('Noise, no signal');


%% SIGNAL PLOT

dataIn = textread('log_signal.csv', '', 'delimiter', ',', ... 
                'emptyvalue', NaN);
            
%% DEFINE FILTER
alpha = 0.5;
throwOut = 100000;

dataOut = [];
for dataSet = 1:length(dataIn(:,1))
    if dataSet > 1
        newDataSet = [];
        for i = 1:length(dataIn(1,:))
            if abs(dataIn(dataSet,i) - dataIn(dataSet-1,i)) > throwOut
                dataIn(dataSet,i) = dataIn(dataSet-1,i);
            end
            newDataPoint = (1-alpha)*dataOut(dataSet-1,i) + (alpha)*(dataIn(dataSet,i));
            newDataSet = [newDataSet newDataPoint];
        end
        dataOut = [dataOut; newDataSet];
    else
        dataOut = [dataOut; dataIn(dataSet,:)];
    end
end
%subplot(2,1,2);
%plot(dataIn(:,1),'.-r');
hold on;
%plot(dataOut(:,1),'-b','LineWidth',2);
xlabel('Sample Number (20 Hz)');
ylabel('Amplitude (dps)');
title('Signal, low noise');