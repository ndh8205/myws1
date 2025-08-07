function [Ad, Bd] = linearize_rocket(X, final_cmd, params, dt)
% LINEARIZE_ROCKET HANul 로켓 선형화 함수
%
% 입력:
%   X         - 현재 상태 벡터 [13x1] [pos; vel; quat; omega]
%   final_cmd - 제어 명령 벡터 [9x1] [F_thrust; canard1-4; RCS1-4]
%   params    - 로켓 파라미터 구조체 (vehicle_params 함수 출력)
%   dt        - 시간 스텝 [s]
%
% 출력:
%   Ad        - 이산 시간 상태 행렬
%   Bd        - 이산 시간 입력 행렬

    % 1. 제어 할당 행렬 (Gamma) 생성 - 실제 동역학 함수와 일치
    d_l = params.vehicle.Lc;               % 카나드 모멘트 암 [m]
    d_r = params.vehicle.r_ref;            % RCS 모멘트 암 반경 [m]
    T_RCS = params.vehicle.RCS_T;          % RCS 추력 [N]
    
    % 카나드 양력 계수 (기본값)
    CL1 = 0.6; CL2 = 0.6; CL3 = 0.6; CL4 = 0.6;
    
    % 추력값 설정 (로켓 추력, N)
    F_T = params.vehicle.thrust;           % 추력값 [N] - 파라미터에서 가져옴
    m = params.vehicle.m_W;                % 질량 [kg]
    g_I = params.environment.g;
    g_I = g_I(3);
    
    % 제어 할당 행렬 생성 (추력 추가로 6x9로 확장)
    Gamma = zeros(6, 9);
    
    % 추력 제어 추가 (첫번째 열)
    Gamma(1, 1) = F_T;                  % F_thrust → Fx/m
    
    % 롤 제어 (RCS)
    Gamma(4, 6) = T_RCS * d_r;            % RCS 1
    Gamma(4, 7) = -T_RCS * d_r;           % RCS 2
    Gamma(4, 8) = T_RCS * d_r;            % RCS 3
    Gamma(4, 9) = -T_RCS * d_r;           % RCS 4
    
    % 피치 제어 (카나드 2, 4만 사용)
    Gamma(5, 3) = -d_l * CL2;             % 카나드 2 (-)
    Gamma(5, 5) = d_l * CL4;              % 카나드 4 (+)
    
    % 요 제어 (카나드 1, 3만 사용)
    Gamma(6, 2) = d_l * CL1;              % 카나드 1 (+)
    Gamma(6, 4) = -d_l * CL3;             % 카나드 3 (-)
    
    % 제어 입력에서 힘/모멘트 계산
    U_FM = Gamma * final_cmd;
    
    % 2. 공력 미분계수 계산
    % 현재 상태에서 공력계수 계산
    % 마하수 추정 (현재 속도 기준)
    V_mag = norm(X(4:6));           % 속도 크기 [m/s]
    
    % 표준 대기 모델에서 음속 계산
    try
        altitude = -X(3);                  % 고도 [m] (Z축 아래 방향 양수)
        [~, sound_speed, ~, rho] = atmosisa(altitude);  % 표준 대기 모델
    catch
        % 대기 모델 실패 시 기본값 사용
        sound_speed = 340;                 % 음속 [m/s]
        rho = 1.225;                       % 공기 밀도 [kg/m^3]
    end
    
    current_mach = V_mag / sound_speed;    % 마하수
    
    % 받음각 추정
    % 속도가 0에 가까울 때 오류 방지
    if V_mag > 1e-3
        % 동체 X축과 속도 벡터 사이의 각도 계산
        % 간단한 근사: atan2(sqrt(v^2+w^2), u)
        body_vel = X(4:6);
        alpha_rad = atan2(sqrt(body_vel(2)^2 + body_vel(3)^2), body_vel(1));
        alpha_deg = rad2deg(alpha_rad);
    else
        alpha_deg = 0;  % 속도가 매우 작을 때 기본값
    end
    
    % 각속도 추출
    p = X(11); q = X(12); r = X(13);
    
    % aero_coefficients 함수 호출하여 미분계수 획득
    try
        aero = aero_coefficients(params, alpha_deg, current_mach, p, q, r, false);
        
        % 속도 미분계수 추출
        if isfield(aero, 'velocity_derivatives')
            CXu = aero.velocity_derivatives.CXu;
            CYv = aero.velocity_derivatives.CYv;
            CZw = aero.velocity_derivatives.CZw;
            Clp = aero.velocity_derivatives.Clp;
            Cmq = aero.velocity_derivatives.Cmq;
            Cnr = aero.velocity_derivatives.Cnr;

            disp('CXu')
            disp(CXu)

            disp('CYv')
            disp(CYv)
            
            disp('CZw')
            disp(CZw)
            
        else
            % 대체값 사용
            disp('대체값 사용')
            CXu = -0.1; CYv = -0.3; CZw = -0.3;
            if isfield(aero, 'damping_derivatives')
                Clp = aero.damping_derivatives.Clpm;
                Cmq = aero.damping_derivatives.Cmqm;
                Cnr = aero.damping_derivatives.Cyawrm;
            else
                Clp = -0.1; Cmq = -0.5; Cnr = -0.5;
            end
        end
    catch
        % 공력 계수 계산 실패 시 기본값 사용
        CXu = -0.1; CYv = -0.3; CZw = -0.3;
        Clp = -0.1; Cmq = -0.5; Cnr = -0.5;
    end
    
    % 3. 공력 기준값 계산
    % 동압
    qbar = 0.5 * rho * V_mag^2;         % 동압 [Pa]
    Sref = params.vehicle.S_A_ref;       % 기준 면적 [m^2]
    Lref = params.vehicle.D_ref;         % 기준 길이 [m]
    
    % 4. 질량 및 관성 모멘트
    J = params.vehicle.J;                % 관성 텐서 [kg*m^2]
    Jxx = J(1,1); Jyy = J(2,2); Jzz = J(3,3);
    
    % 5. 심볼릭 선형화 함수 호출 - 이산 행렬 획득
    
    % 힘/모멘트 기반 기존 함수 사용 - 이산 행렬 반환
    [Ad_disc, Bd_disc] = hanul_linear_AB_zoh(X, U_FM, dt, ...
        m, Jxx, Jyy, Jzz, g_I,...
        CXu, 0, 0, Clp, Cmq, Cnr, ...
        qbar, Sref, Lref, ...
        d_l, d_r, T_RCS, CL1, CL2, CL3, CL4, F_T);
    
    % Bd 행렬이 힘/모멘트 기준이면 액추에이터 기준으로 변환
    if size(Bd_disc, 2) == 6  % 6열이면 힘/모멘트 기준
        Bd_disc = Bd_disc * Gamma;
    end
    
    Ad = Ad_disc;
    Bd = Bd_disc;
end