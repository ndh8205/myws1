
function H = calcHMatrix_Linear(xhat, X_target)


    % n = size(xhat);
    % 
    % H = zeros( n(1),n(1) );
    % 
    % H(1,1) = 1; % pos
    % H(2,2) = 1; % pos
    % H(3,3) = 1; % pos
    % % H(4,4) = 0;
    % % H(5,5) = 0;
    % % H(6,6) = 0;
    % % H(7,7) = 0;
    % % H(8,8) = 0;
    % % H(9,9) = 0;
    % H(10,10) = 1; % rate
    % H(11,11) = 1; % rate
    % H(12,12) = 1; % rate
    % % H(13,13) = 0;
    % % H(14,14) = 0;
    % % H(15,15) = 0;
    % % H(16,16) = 0;

    H = zeros(6,13);
    H(1,1) = 1; % pos
    H(2,2) = 1; % pos
    H(3,3) = 1; % pos
    H(4,11) = 1; % rate
    H(5,12) = 1; % rate
    H(6,13) = 1; % rate

end