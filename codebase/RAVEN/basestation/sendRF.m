function sendRF(serialID,packet)
    for i = 1:length(packet)
        fwrite(serialID,packet(i),'uchar');
    end
end