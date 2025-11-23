% 데이터 수신 함수
function data = receiveDataFromJetson(client)
    data = '';
    try
        while client.BytesAvailable > 0
            data = [data, char(read(client, client.BytesAvailable))'];
        end
    catch
        disp('Failed to receive data from Jetson.');
    end
end