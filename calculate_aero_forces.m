function [F_b, M_b] = calculate_aero_forces(static_coeffs, damping_moments, missile_params, phi_A)
% calculate_aero_forces  공력계수에서 힘과 모멘트 계산
%
% [F_b, M_b] = calculate_aero_forces(static_coeffs, damping_moments, missile_params, phi_A)
%
% 입력:
% static_coeffs : 정적 공력계수 구조체 (CAm, CYm, CNm, Clm, Cmm, Cyawm)
% damping_moments : 감쇠 모멘트 계수 구조체 (Clmd, Cmmd, Cyawmd)
% missile_params : 미사일 파라미터 구조체
% phi_A : 에어로 롤각 [라디안]
%
% 출력:
% F_b : 바디 프레임에서의 공력 힘 벡터 (3x1) [N]
% M_b : 바디 프레임에서의 공력 모멘트 벡터 (3x1) [N-m]

% 필요한 미사일 파라미터 추출
Q = 0.5 * missile_params.environment.ro * missile_params.VR^2; % 동압 [Pa]
Sref = missile_params.vehicle.S_A_ref; % 참조 면적 [m^2]
Dref = missile_params.vehicle.D_ref;   % 참조 길이 [m]

% 1. M 프레임에서의 힘 계산 (NASA문서 식 10)
F_m = Q * Sref * [-static_coeffs.CAm; 
                   static_coeffs.CYm; 
                  -static_coeffs.CNm];

% 2. M 프레임에서의 모멘트 계산 (NASA문서 식 11)
M_m = Q * Sref * Dref * [static_coeffs.Clm + damping_moments.Clmd; 
                         static_coeffs.Cmm + damping_moments.Cmmd; 
                         static_coeffs.Cyawm + damping_moments.Cyawmd];

% 3. M 프레임에서 B 프레임으로 변환
c_phi = cos(phi_A);
s_phi = sin(phi_A);
M_to_B = [1, 0, 0;
          0, c_phi, s_phi;
          0, -s_phi, c_phi];

F_b = M_to_B * F_m;
M_b = M_to_B * M_m;

% 디버그 정보 출력 (필요시 주석 해제)
% fprintf('계산된 공력 정보:\n');
% fprintf('동압(Q): %.2f Pa\n', Q);
% fprintf('힘(F): [%.2f, %.2f, %.2f] N\n', F_b(1), F_b(2), F_b(3));
% fprintf('모멘트(M): [%.2f, %.2f, %.2f] N-m\n', M_b(1), M_b(2), M_b(3));
end