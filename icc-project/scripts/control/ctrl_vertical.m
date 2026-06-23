function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [학생 작성] CDC (Continuous Damping Control) — per-wheel 감쇠 명령
%
%   Body-bounce / wheel-hop 모드 분리 및 ride comfort 개선을 위한 가변 감쇠.
%
%   Inputs:
%       suspState - struct, 각 wheel 의 sprung/unsprung velocity 등
%           .zs_dot(4)     - sprung mass velocity (위쪽 양수) [m/s]
%           .zu_dot(4)     - unsprung mass velocity [m/s]
%           .zs(4), .zu(4) - 변위 [m]
%       ctrlState - 내부 상태
%       CTRL      - .VER.cMin (≈ 500), .cMax (≈ 5000), .skyGain (≈ 2500)
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4×1 damping coefficient [Ns/m]
%
%   요구사항:
%       1. Skyhook 기본:  c_i = skyGain · sign(zs_dot_i · (zs_dot_i - zu_dot_i))
%          (또는 force form: F = skyGain · zs_dot, F = c · (zs_dot - zu_dot))
%       2. cMin ≤ c ≤ cMax 제한
%       3. (옵션) Hybrid skyhook + groundhook
%       4. (옵션) body-bounce/wheel-hop 빈도 분리
%
%   힌트:
%       - Skyhook 의 핵심 원리: sprung mass 가 절대 좌표에서 정지하길 원함 → relative
%         damping 을 변조해 sprung velocity 를 줄임.
%       - 간단 force version: 항상 c = c_nom 으로 두고, (zs_dot · (zs_dot - zu_dot)) > 0
%         일 때만 c = cMax, 아니면 c = cMin (semi-active 의 on-off skyhook).

    %% TODO: 학생 구현
    %  (1) skyhook (또는 변형)
    %  (2) per-wheel 적용
    %  (3) cMin/cMax 제한
%% 1. 입력 예외 처리
if ~isfield(suspState, 'zs_dot') || isempty(suspState.zs_dot)
    dampingCmd = CTRL.VER.cMin * ones(4,1);
    return;
end

zs_dot = suspState.zs_dot;
zu_dot = suspState.zu_dot;
dampingCmd = zeros(4,1);

%% 2. Roll Rate 추정
roll_rate_f = zs_dot(1) - zs_dot(2);
roll_rate_r = zs_dot(3) - zs_dot(4);

roll_threshold = 0.08;
is_rolling = (abs(roll_rate_f) > roll_threshold) || (abs(roll_rate_r) > roll_threshold);

%% 3. Skyhook 기반 감쇠 계수 계산
for i = 1:4
    rel_vel = zs_dot(i) - zu_dot(i);

    if zs_dot(i) * rel_vel > 0
        c = CTRL.VER.skyGain * 5.0 * abs(zs_dot(i)) / max(abs(rel_vel), 0.001);
    else
        c = CTRL.VER.cMin;
    end

    %% 4. Roll 발생 시 감쇠 강화
    if is_rolling
        c = CTRL.VER.cMax;
    end

    %% 5. 감쇠 계수 제한
    dampingCmd(i) = max(CTRL.VER.cMin, min(CTRL.VER.cMax, c));

end