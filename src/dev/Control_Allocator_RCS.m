function [final_cmd, U_actuator] = Control_Allocator_RCS(U_control, params)
    % 4개 추력기와 4개 카나드 기반 제어 할당기
    % U_control: 6x1 제어 입력 벡터(모멘트) - X Y Z 롤, 피치, 요
    % params: 로켓 파라미터 구조체
    % 출력:
    % final_cmd: 8x1 최종 제어 명령 벡터 (카나드 δ₁~δ₄, 추력기 T₁~T₄)
    % U_actuator: 8x1 액츄에이터 명령 벡터 (스케일링 전)
    
    
    % 로켓 제어 파라미터 추출
    Jxx = params.vehicle.J_X;  % X축 관성 모멘트 [kgm^2]
    Jyy = params.vehicle.J_Y;  % Y축 관성 모멘트 [kgm^2]
    Jzz = params.vehicle.J_Z;  % Z축 관성 모멘트 [kgm^2]
    d = params.vehicle.r_ref;  % RCS 반경 거리 [m] (CG to Thruster - Y&Z)
    l = params.vehicle.Lrcs;   % RCS 축 방향 거리 [m] (CG to Thruster - X)
    m = params.vehicle.m_W;    % 현재 로켓 질량 [kg]
    T = params.vehicle.RCS_T;  % RCS 추력 [N]
    
    % 제어 이득
    M = T/m;                   % 힘/질량 [m/s^2]
    R_RCS = (T*d)/Jxx;         % 롤 모멘트/관성모멘트 [rad/s^2]
    P_RCS = (T*l)/Jyy;         % 피치 모멘트/관성모멘트 [rad/s^2]
    Y_RCS = (T*l)/Jzz;         % 요 모멘트/관성모멘트 [rad/s^2]
    
    % 카나드 효과 계수 (추정값, 실제 값으로 대체 필요)
    eps1 = 1.0;  % Z축 모멘트에 대한 카나드 1의 효과
    eps2 = 1.0;  % Y축 모멘트에 대한 카나드 2의 효과
    eps3 = 1.0;  % Z축 모멘트에 대한 카나드 3의 효과
    eps4 = 1.0;  % Y축 모멘트에 대한 카나드 4의 효과
    
    % 카나드 모멘트 효과 계산 (이 값들은 실제 공력 데이터로 조정해야 함)
    C_pitch = (eps2 * l) / Jyy;  % 피치 제어 효과
    C_yaw = (eps1 * l) / Jzz;    % 요 제어 효과
    
    % 제어 할당 행렬 - 제시된 행렬 B 기반
    % [δ₁, δ₂, δ₃, δ₄, T₁, T₂, T₃, T₄]
    A_total_cmd = [
           1/m, 0, 0, 0, 0, 0, 0, 0, 0;  % Fx - 모두 0으로 설정
             0, 0, 0, 0, 0, 0, 0, 0, 0;  % Fy - 모두 0으로 설정
             0, 0, 0, 0, 0, 0, 0, 0, 0;  % Fz - 모두 0으로 설정
             0, 0, 0, 0, 0, R_RCS, -R_RCS, R_RCS, -R_RCS;  % Roll (롤) - 추력기
             0, 0, C_pitch, 0, -C_pitch, 0, 0, 0, 0;       % Pitch (피치) - 카나드
             0, C_yaw, 0, -C_yaw, 0, 0, 0, 0, 0;           % Yaw (요) - 카나드
    ];
    
    % 제어 입력을 액츄에이터 명령으로 변환
    % 6x1 제어 벡터 -> 9x1 액츄에이터 명령 벡터
    U_actuator = pinv(A_total_cmd) * U_control;
    
    % 액츄에이터 제한
    for i = 1:9
        U_actuator(i) = LIMIT2(U_actuator(i), -20, 20);
    end
    
    % 최종 명령 벡터 초기화
    final_cmd = zeros(9, 1);
    final_cmd(1) = U_control(1);
    
    % 카나드 명령 (δ₁, δ₂, δ₃, δ₄) - 연속적인 각도 값
    final_cmd(2:5) = U_actuator(2:5);
    
    % 추력기 명령 (T₁, T₂, T₃, T₄) - 이진 명령 (0 또는 1)
    thruster_commands = U_actuator(6:9);
    thruster_binary = thruster_commands > 0;  % 양수면 1, 아니면 0
    final_cmd(6:9) = thruster_binary;
end