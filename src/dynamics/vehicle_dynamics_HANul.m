function [xdot, Thrust_mass_vec, alpha_tot, phi_A, F_b, M_b, moment_coupling] = vehicle_dynamics_HANul(X, U, Qk, dt, params, t)
    % 새로운 파라미터 구조로 업데이트된 로켓 6-DOF 동역학 함수
    % NASA "Missile Aerodynamics for Ascent and Re-entry" 문서에 기반한 구현
    % 카나드 제어 추가 버전
    persistent on_launch_rod previous_altitude

    % 로켓 기하학적 파라미터 추출
    S_A_ref = params.vehicle.S_A_ref;   % 로켓 단면적 [m^2]
    D_ref = params.vehicle.D_ref;       % 로켓 직경 [m]
    J = params.vehicle.J;               % 관성 텐서 [kg*m^2]
    
    % 현재 질량 및 추력
    m = params.vehicle.m_W;             % 현재 질량 [kg]
    current_thrust = params.vehicle.thrust;  % 현재 추력 [N]
    
    % RCS 파라미터
    L_RCS = params.vehicle.Lrcs;        % CG에서 RCS까지 거리 [m]
    T_RCS = params.vehicle.RCS_T;       % RCS 추력 [N]
    D_RCS = params.vehicle.r_ref;       % 로켓 반경 [m]
    
    % 카나드 파라미터
    L_Canard = params.vehicle.Lc;       % CG에서 카나드까지 거리 [m]
    
    % 카나드 효과 계수
    eps1 = 1.0;  % Z축 모멘트에 대한 카나드 1의 효과
    eps2 = 1.0;  % Y축 모멘트에 대한 카나드 2의 효과
    eps3 = 1.0;  % Z축 모멘트에 대한 카나드 3의 효과
    eps4 = 1.0;  % Y축 모멘트에 대한 카나드 4의 효과
    
    % 모멘트 기준점(MRP) 좌표 추출
    MRP_x = params.vehicle.MRP_x;       % 설계 좌표계에서의 MRP X 위치
    MRP_y = params.vehicle.MRP_y;       % 설계 좌표계에서의 MRP Y 위치
    MRP_z = params.vehicle.MRP_z;       % 설계 좌표계에서의 MRP Z 위치
    
    % CG 위치 (설계 좌표계)
    CG_x = params.vehicle.CG_abs(1);
    CG_y = params.vehicle.CG_abs(2);
    CG_z = params.vehicle.CG_abs(3);
    
    % MRP를 바디 프레임(CG 기준)으로 변환 - MRP_b 벡터 계산
    MRP_b_x = (CG_x - MRP_x);
    MRP_b_y = (CG_y - MRP_y); 
    MRP_b_z = (CG_z - MRP_z);
    
    % MRP_b 벡터 (CG에서 MRP까지의 벡터) - 수정: 부호 일관성 유지
    MRP_b = [MRP_b_x; MRP_b_y; MRP_b_z];
    
    % 환경 파라미터
    g = params.environment.g;           % 중력 가속도 [m/s^2]
    
    % 상태 벡터 분해
    pos = X(1:3);      % 위치 (X Y Z) - 관성 좌표계
    V_B = X(4:6);      % 속도 (u v w) - 바디 좌표계
    att = X(7:10);     % 자세 (q0 q1 q2 q3) - 관성 좌표계
    omega_b = X(11:13);  % 각속도 (p q r) - 바디 좌표계
    
    % 발사대 파라미터 설정
    launch_rod_length = 1.0;  % 발사대 길이 [m]
    
    % 발사대 탈출 감지 로직
    if isempty(on_launch_rod)
        on_launch_rod = true;
        previous_altitude = -pos(3);
    end
    
    % 로켓이 발사대를 떠났는지 확인
    has_left_launch_rod = (-pos(3) > launch_rod_length);
    
    % 발사대 탈출 속도 표시 (발사대를 처음 벗어날 때)
    if on_launch_rod && has_left_launch_rod
        rocket_speed = norm(V_B);
        disp(['발사대 탈출! 탈출 속도: ', num2str(rocket_speed * 3.6), ' km/h']);
        on_launch_rod = false;
    end
    
    % 고도 저장 (다음 단계 비교용)
    previous_altitude = -pos(3);
    
    % 프로세스 노이즈 벡터 분해
    Fx_Q = Qk(1);    % 병진 불확실성
    Fy_Q = Qk(2);
    Fz_Q = Qk(3);
    Rx_Q = Qk(4);    % 회전 불확실성
    Py_Q = Qk(5);
    Yz_Q = Qk(6);
    
    % 대기 속성 계산 (ISA 모델 사용)
    [Temp, a, Pa, rho] = atmosisa(-pos(3));
    ro = rho;
    
    % 추력 및 질량 벡터 설정
    F_T = [current_thrust, 0, 0]';
    Thrust_mass_vec = [F_T; m];
    
    % 회전 행렬 계산
    R_B2I_q = GetDCM_QUAT(att);
    R_I2B_q = R_B2I_q';
    
    % 바람 효과 추가
    persistent wind_time wind_speed
    if isempty(wind_time) || t >= wind_time + 1/400  % 20Hz 업데이트
        wind_time = t;
        % 바람 정보 추출
        if isfield(params.environment, 'wind_speed')
            base_wind_speed = params.environment.wind_speed;
            if isfield(params.environment, 'wind_turbulence')
                turb_intensity = params.environment.wind_turbulence;
            else
                turb_intensity = 0.1;  % 기본 난류 강도
            end
        else
            base_wind_speed = 5.0;  % 기본 풍속 [m/s]
            turb_intensity = 0.1;   % 기본 난류 강도
        end
        
        % 핑크 노이즈를 이용한 바람 생성
        wind_turb = randn() * base_wind_speed * turb_intensity;
        wind_speed = base_wind_speed + wind_turb;
    end
    
    % 바람 방향 설정
    if isfield(params.environment, 'wind_direction')
        wind_direction = params.environment.wind_direction;
    else
        wind_direction = 0;  % 기본값 (동쪽)
    end
    
    % 바람을 바디 좌표계로 변환
    wind_I = [wind_speed * cos(wind_direction);
              wind_speed * sin(wind_direction);
              0];
    V_wind_B = R_I2B_q * wind_I;
    V_B_aero = V_B - V_wind_B;  % 공력 계산용 상대 속도
    
    % ----- M-프레임 관련 계산 (NASA 문서 참조) -----
    % αtot 및 φA 계산
    VR = norm(V_B_aero);
    if VR < 1e-8
        alpha_tot = 0;
        phi_A = 0;
    else
        % 식(1): αtot = cos^-1(VRx/VR)
        alpha_tot = acos(V_B_aero(1) / VR);  % 총 받음각 [rad]
        
        % 식(2): φA = tan^-1(VRy/VRz)
        phi_A = atan2(V_B_aero(2), V_B_aero(3));  % 공력 롤 각도 [rad]
        
        % αtot=0° 또는 180°인 경우의 특이점 처리
        if abs(alpha_tot) < 1e-6 || abs(alpha_tot - pi) < 1e-6
            phi_A = 0;
        end
    end
    
    % 마하수 계산 - 식(3)
    mach = VR / a;
    
    % Body 프레임에서 M-프레임으로의 회전 행렬 계산
    % φA에 대한 회전 행렬 (X축 중심 회전)
    R_B2M = [1,       0,          0;
             0, cos(phi_A), -sin(phi_A);
             0, sin(phi_A),  cos(phi_A)];
    
    % 각속도를 M-프레임으로 변환
    omega_m = R_B2M * omega_b;
    
    % M-프레임에서의 각속도 성분
    pm = omega_m(1);  % 롤 각속도 [rad/s]
    qm = omega_m(2);  % 피치 각속도 [rad/s]
    rm = omega_m(3);  % 요 각속도 [rad/s]
    
    % 각도를 도(degree)로 변환 (aero_coefficients 함수 입력용)
    alpha_tot_deg = alpha_tot * 180/pi;  % 총 받음각 [deg]
    
    % 공력 계수 계산 (aero_coefficients 함수 사용)
    aero = aero_coefficients(params, alpha_tot_deg, mach, pm, qm, rm, false);
    
    % 공력 계수 추출 및 수정 (스케일링을 통한 안정성 향상)
    CAm = aero.static.CAm;     
    CYm = aero.static.CYm;   
    CNm = aero.static.CNm;  
    Clm = aero.static.Clm; 
    Cmm = aero.static.Cmm;  
    Cyawm = aero.static.Cyawm;
    
    % 댐핑 계수 추출 및 강화
    if isfield(aero.damping_derivatives, 'Clpm')
        Clpm = aero.damping_derivatives.Clpm;  % 롤 댐핑 미분계수 강화
    else
        Clpm = -0.9;  % 기본값 강화
    end
    
    if isfield(aero.damping_derivatives, 'Cmqm')
        Cmqm = aero.damping_derivatives.Cmqm;  % 피치 댐핑 미분계수 강화
    else
        Cmqm = -1.0;  % 기본값 강화
    end
    
    if isfield(aero.damping_derivatives, 'Cyawrm')
        Cyawrm = aero.damping_derivatives.Cyawrm;  % 요 댐핑 미분계수 강화
    else
        Cyawrm = -1.0;  % 기본값 강화
    end
    
    % 댐핑 모멘트 계수 계산 (NASA 문서 식 7-9)
    if VR > 1e-3  % 속도가 0에 가까울 때 발산 방지
        Clmd = (pm * D_ref / (2 * VR)) * Clpm;
        Cmmd = (qm * D_ref / (2 * VR)) * Cmqm;
        Cyawmd = (rm * D_ref / (2 * VR)) * Cyawrm;
    else
        Clmd = 0;
        Cmmd = 0;
        Cyawmd = 0;
    end
    
    % 동압 계산
    q_aero = 0.5 * ro * VR^2;  % 동압
    
    % ----- M-프레임에서 공력 힘 및 모멘트 계산 -----
    % NASA 문서 식(10): M-프레임에서의 공력 힘
    F_m = q_aero * S_A_ref * [CAm; CYm; CNm];
    
    % NASA 문서 식(11): M-프레임에서의 공력 모멘트
    M_m = q_aero * S_A_ref * D_ref * [Clm + Clmd; Cmm + Cmmd; Cyawm + Cyawmd];
    
    % ----- M-프레임에서 P-프레임으로 변환 -----
    % NASA 문서 식(12): 힘 변환 - 회전 행렬 수정
    R_M2P = [1,        0,         0;
             0,  cos(phi_A), sin(phi_A);
             0, -sin(phi_A), cos(phi_A)];
    
    F_p = R_M2P * F_m;
    
    % NASA 문서 식(13): 모멘트 변환
    M_p = R_M2P * M_m;
    
    % ----- P-프레임에서 B-프레임으로 변환 -----
    % NASA 문서 식(14): P-프레임 힘을 B-프레임으로 (축이 평행하므로 동일)
    F_b = F_p;
    
    % NASA 문서 식(15): B-프레임에서의 모멘트
    M_b = M_p + cross(MRP_b, F_p);
    moment_coupling = cross(MRP_b, F_p);
  
    % ----- 제어 입력 처리 (통합된 카나드 및 RCS) -----
    % 제어 명령 벡터 분해 (8x1 벡터)
    % U의 형식: [카나드1, 카나드2, 카나드3, 카나드4, 추력기1, 추력기2, 추력기3, 추력기4]
    canard_angles = U(1:4);  % 카나드 각도 명령 (4개)
    thruster_cmds = U(5:8);  % 추력기 명령 (4개)
    
    % 카나드 각도 제한 (±5도)
    canard_max_angle = deg2rad(5);  % 최대 5도
    canard_min_angle = deg2rad(-5); % 최소 -5도
    
    % 각도 제한 적용
    for i = 1:4
        canard_angles(i) = max(canard_min_angle, min(canard_max_angle, canard_angles(i)));
    end
    
    % 카나드 공력 계수 설정 (대칭 에어포일 선형 모델)
    Cl_alpha = 0.6;  % 양력 기울기 (이론적인 2π ≈ 6.28)
    canard_area = 0.01; % 각 카나드 면적 [m^2]
    
    % 카나드 효과 계산 - 각도에 따른 양력 계수
    % 대칭 에어포일이므로 선형 모델: Cl = Cl_alpha * angle
    canard_cl = Cl_alpha * canard_angles;
    
    % 양력 계산 (동압에 비례)
    canard_lift = q_aero * canard_area * canard_cl;
    
    % 최대 양력 제한 (3N)
    MAX_LIFT = 10; % 최대 양력 [N]
    for i = 1:4
        if abs(canard_lift(i)) > MAX_LIFT
            canard_lift(i) = sign(canard_lift(i)) * MAX_LIFT;
        end
    end
    
    % 카나드 모멘트 계산
    M_canard = zeros(3, 1);
    % 롤: 사용하지 않음 (RCS가 담당)
    M_canard(1) = 0;
    % 피치: 카나드 2(-), 4(+)가 생성
    M_canard(2) = (canard_lift(2) * eps2 - canard_lift(4) * eps4) * L_Canard;
    % 요: 카나드 1(+), 3(-)가 생성
    M_canard(3) = (canard_lift(1) * eps1 - canard_lift(3) * eps3) * L_Canard;
    
    % RCS 제어 효과 계산
    M_rcs = zeros(3, 1);
    % 롤: 추력기만 사용
    M_rcs(1) = T_RCS * D_RCS * (thruster_cmds(1) - thruster_cmds(2) + thruster_cmds(3) - thruster_cmds(4));
    % 피치와 요: 사용하지 않음 (카나드가 담당)
    M_rcs(2) = 0;
    M_rcs(3) = 0;
    
    % 제어력과 모멘트 결합
    Force_control = zeros(3, 1);  % 병진력 없음
    Torque_control = M_canard + M_rcs;  % 카나드와 RCS의 모멘트 결합
    
    % ----- 최종 힘 및 모멘트 계산 (B-프레임) -----
    % 중력 힘 (바디 좌표계)
    F_G = R_I2B_q * (m*g);
    
    % 추력 벡터 (바디 좌표계)
    F_T_body = [current_thrust; 0; 0];
    
    % 발사대 효과 적용 - 레일에 있을 때는 추력 외 모든 힘과 모멘트 제한
    if ~has_left_launch_rod
        % 1. 공력 힘 제한 - X 방향(축력)만 유지, Y/Z 방향 힘은 제거
        F_b = [F_b(1); 0; 0];
        
        % 2. 제어 힘과 모멘트 제거 (레일에서는 제어 효과 없음)
        Force_control = [0; 0; 0];
        Torque_control = [0; 0; 0];
        
        % 3. 중력 X 성분만 유지, Y/Z 성분은 레일이 지지
        F_G = [F_G(1); 0; 0];
        
        % 4. 모든 모멘트 제거 (레일이 회전 구속)
        M_b = [0; 0; 0];
        
        % 5. 노이즈도 제거
        Fx_Q = 0;
        Fy_Q = 0;
        Fz_Q = 0;
        Rx_Q = 0;
        Py_Q = 0;
        Yz_Q = 0;
    end
    
    % 총 힘 (공력 + 중력 + 추력 + 제어력 + 노이즈)
    F_total = F_b + F_G + F_T_body + Force_control + [Fx_Q; Fy_Q; Fz_Q];
    
    % 총 모멘트 (공력 + 제어 모멘트 + 노이즈)
    M_total = M_b + Torque_control + [Rx_Q; Py_Q; Yz_Q];
    
    % ----- 로켓 상태 방정식 -----
    att = att / norm(att);  % 쿼터니언 정규화
    
    % 발사대에 있을 때 추가 구속 (직선 운동만 허용)
    if ~has_left_launch_rod
        % 1. 속도 성분 제한 (X방향만 허용)
        V_B = [V_B(1); 0; 0];
        
        % 2. 각속도 제한 (모든 회전 금지)
        omega_b = [0; 0; 0];
    end
    
    x1_dot = R_B2I_q * V_B;                        % 위치 미분 - 관성 좌표계
    x2_dot = (F_total / m) - cross(omega_b, V_B);  % 속도 미분 - 바디 좌표계
    
    % 발사대에 있을 때 추가 구속
    if ~has_left_launch_rod
        % X 방향 속도만 허용 (Y, Z 성분 제거)
        x2_dot = [x2_dot(1); 0; 0];
    end
    
    x3_dot = Derivative_Quat(att, omega_b);        % 자세 미분 - 관성 좌표계
    x4_dot = inv(J) * (M_total - cross(omega_b, J*omega_b));  % 각속도 미분 - 바디 좌표계
    
    % 발사대에 있을 때 각속도 변화 제한
    if ~has_left_launch_rod
        x4_dot = [0; 0; 0];  % 각속도 변화 없음
    end
    
    xdot = [x1_dot; x2_dot; x3_dot; x4_dot];
end