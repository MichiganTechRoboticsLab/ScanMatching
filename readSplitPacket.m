function [ recievedBytes, num, finished ] = readSplitPacket( fs )
%READMULTIPACKET Summary of this function goes here
%   Detailed explanation goes here

loop = 1;
num = 0;
recievedBytes = [];
while( loop == 1)
    [ in, count ] = fread(fs);
    str = char(in');
    switch str
        case 'BEGIN'
            fprintf('Start Creating Packet\n')
            num = 0;
            recievedBytes = [];
        case 'END'
            fprintf('Stop reading Packet\n')
            loop = 0;
            finished = 0;
        case 'DONE'
            fprintf('No More Packets\n')
            loop = 0;
            finished = 0;
        otherwise
            num = num + count;
            recievedBytes = cat(1,recievedBytes, in);
    end
end

end

