close all
clear all
clc

Test_Quaternion_Euler_Conversion()

% 특이점 복원 테스트 메인 코드 (도 단위로 표시)
function Test_Quaternion_Euler_Conversion()
    % GetDCM_QUAT 함수 (쿼터니언에서 DCM으로 변환)
    % 원래 코드에 없으므로 추가했습니다
    function A = GetDCM_QUAT(q)
        qw = q(1); qx = q(2); qy = q(3); qz = q(4);
        A = [1-2*(qy^2+qz^2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
             2*(qx*qy+qw*qz), 1-2*(qx^2+qz^2), 2*(qy*qz-qw*qx);
             2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(qx^2+qy^2)];
    end

    % 테스트 케이스 정의 (요, 피치, 롤 순서) - 도 단위
    test_cases_deg = {
        [11.46, 90, 5.73],      % 정확히 90도 피치
        [11.46, 89.9, 5.73],    % 약 89.9도 피치
        [11.46, -90, 5.73],     % 정확히 -90도 피치
        [11.46, -89.9, 5.73],   % 약 -89.9도 피치
        [40.11, 28.65, 17.19]   % 일반 각도
    };
    
    % 테스트 케이스 이름
    case_names = {
        '정확히 90도 피치',
        '약 89.9도 피치',
        '정확히 -90도 피치',
        '약 -89.9도 피치',
        '일반 각도'
    };
    
    fprintf('=== 쿼터니언-오일러 변환 테스트 (Yaw-Pitch-Roll 시퀀스) ===\n');
    fprintf('각도는 도(degree) 단위로 표시됩니다.\n\n');
    
    % 각 테스트 케이스에 대해 변환 수행
    for i = 1:length(test_cases_deg)
        fprintf('테스트 %d: %s\n', i, case_names{i});
        
        % 도->라디안 변환 (내부 계산용)
        original_euler_deg = test_cases_deg{i}';  % [Yaw, Pitch, Roll]
        original_euler_rad = original_euler_deg * (pi/180); % 도->라디안
        
        fprintf('원래 오일러 각도(deg) [Yaw, Pitch, Roll]: [%.2f, %.2f, %.2f]\n', ... 
                original_euler_deg(1), original_euler_deg(2), original_euler_deg(3));
        
        % 오일러 -> 쿼터니언 변환 (Yaw, Pitch, Roll 순서로 전달)
        quat = GetQUAT(original_euler_rad(1), original_euler_rad(2), original_euler_rad(3));
        fprintf('쿼터니언: [%.4f, %.4f, %.4f, %.4f]\n', ...
                quat(1), quat(2), quat(3), quat(4));
        
        % 쿼터니언 -> 오일러 변환
        recovered_euler_rad = Quat2Euler(quat);
        recovered_euler_deg = recovered_euler_rad * (180/pi); % 라디안->도
        
        fprintf('복원된 오일러 각도(deg) [Yaw, Pitch, Roll]: [%.2f, %.2f, %.2f]\n', ...
                recovered_euler_deg(1), recovered_euler_deg(2), recovered_euler_deg(3));
        
        % 오차 계산 (도 단위)
        error_deg = abs(original_euler_deg - recovered_euler_deg);
        fprintf('오차(deg): [%.2f, %.2f, %.2f]\n\n', ...
                error_deg(1), error_deg(2), error_deg(3));
    end
end

function att_euler = Quat2Euler(att_quat)
    % Quaternion to Euler angle (Yaw-Pitch-Roll 시퀀스)
    % Convert quaternion to DCM
    A = GetDCM_QUAT(att_quat);
    
    % 쿼터니언에서 직접 피치 부호 정보 얻기
    qw = att_quat(1);
    qx = att_quat(2);
    qy = att_quat(3);
    qz = att_quat(4);
    
    % Extract Euler angles based on the rotation sequence (Yaw-Pitch-Roll)
    % 피치 각도가 +/- 90도에 가까운지 확인 (특이점 조건)
    if abs(A(3,1)) > 0.99995
        % 특이점 근처 (피치가 +/- 90도에 가까움)
        % 부호 결정 (쿼터니언 값 기반)
        % 쿼터니언의 값으로 원래 피치 부호 결정
        pitch_sign = -sign(A(3,1));
        
        % 수정된 행렬 요소로 요우와 롤 계산
        A(3,1) = pitch_sign * 0.99995;
        
        % 요우와 롤 계산
        psi = atan2(A(2,1), A(1,1)); % Yaw
        phi = atan2(A(3,2), A(3,3)); % Roll
        
        % 피치를 정확히 +/- 90도로 설정
        theta = pitch_sign * pi/2;
    else
        % 일반적인 경우, 특이점 없음
        psi = atan2(A(2,1), A(1,1)); % Yaw
        theta = -asin(A(3,1)); % Pitch
        phi = atan2(A(3,2), A(3,3)); % Roll
    end
    
    % Yaw-Pitch-Roll 순서로 반환
    att_euler = [psi; theta; phi];
end