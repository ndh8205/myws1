clear all
clc
close all

% 로켓 성능 계산
% 상수 및 입력값 정의
ge = 9.81;  % 중력 가속도 (m/s^2)
R = 6/5.6;  % R 값
ue = 1469;  % 배기 속도 (m/s)
tb = 3.8;     % 연소 시간 (s)
t = 3.8;

% 시간에 따른 속도 계산 함수
ub = -ue * log( 1 - (1 - 1/R) * t / tb ) - ge * t;

% hb 계산
hb = -ue * tb * log(R) / (R - 1) + ue * tb - 0.5 * ge * tb^2;

% 최대 고도 계산 (근사치)
hmax = hb + (ub^2 / (2 * ge));

% 결과 출력
fprintf('(hb): %.2f m\n', hb);
fprintf('(hmax): %.2f m\n ', hmax);
fprintf('연소 종료 시 속도 (ub): %.2f m/s\n', ub);
