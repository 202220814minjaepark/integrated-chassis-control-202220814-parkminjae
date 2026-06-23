function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% TODO: 여기에 학생 구현
    %  (1) speed-tracking PI
    %  (2) ABS modulation (이번 함수에서 또는 ctrl_coordinator 에서)
    %  (3) jerk limit
    %  (4) anti-windup
%% 1. 입력 예외 처리 및 내부 상태 초기화
if nargin < 7 || isempty(dt) || ~isscalar(dt) || ~isfinite(dt) || dt <= 0
    dt = 0.001;
end
dtSafe = max(dt, 1e-6);

if isempty(ctrlState) || ~isstruct(ctrlState)
    ctrlState = struct();
end

if ~isfield(ctrlState, 'prevForce')
    ctrlState.prevForce = 0;
end
if ~isfield(ctrlState, 'absScale')
    ctrlState.absScale = 0.35;
end
if ~isfield(ctrlState, 'intError')
    ctrlState.intError = 0;
end

forceCmd.Fx_total   = 0;
forceCmd.brakeRatio = 0;

%% 2. 입력 신호 보호
if isempty(vxRef) || ~isscalar(vxRef) || ~isfinite(vxRef)
    vxRef = 0;
end
if isempty(vx) || ~isscalar(vx) || ~isfinite(vx)
    vx = 0;
end
if isempty(ax) || ~isscalar(ax) || ~isfinite(ax)
    ax = 0;
end

vxSafe = max(vx, 0.1);

%% 3. Wheel Slip 계산
if isfield(ctrlState, 'wheelSlip') && ~isempty(ctrlState.wheelSlip)
    slip = ctrlState.wheelSlip(:);
else
    slip_est = max(-ax, 0) / 9.81 * 0.15;
    slip = slip_est * ones(4, 1);
end

if numel(slip) < 4
    slip = [slip; zeros(4 - numel(slip), 1)];
elseif numel(slip) > 4
    slip = slip(1:4);
end

slip(~isfinite(slip)) = 0;

slipAbs = abs(slip);
slipAbsMax  = max(slipAbs);
slipAbsMean = mean(slipAbs);

slipIndex = 0.70 * slipAbsMax + 0.30 * slipAbsMean;

%% 4. 제동 제한값 설정
m = 1800;     % vehicle mass approximate [kg]

if isfield(LIM, 'MAX_JERK') && isfinite(LIM.MAX_JERK) && LIM.MAX_JERK > 0
    maxJerk = LIM.MAX_JERK;
else
    maxJerk = 15;
end

maxAddBrakeForce = 0.20 * m * 9.81;

%% 5. 제동 상황 감지 및 추가 제동력 계산
brakingLike = (ax < -0.15) || (slipAbsMax > 0.025);

Fx_cmd = 0;

if brakingLike && vxSafe > 3.0

    slipLow  = 0.06;
    slipMid  = 0.10;
    slipHigh = 0.16;

    if slipIndex > slipHigh
        ctrlState.absScale = 0.20 * ctrlState.absScale;

    elseif slipIndex > slipMid
        ctrlState.absScale = 0.60 * ctrlState.absScale;

    elseif slipIndex < slipLow
        ctrlState.absScale = ctrlState.absScale + 0.035;

    else
        ctrlState.absScale = 0.98 * ctrlState.absScale;
    end

    ctrlState.absScale = max(min(ctrlState.absScale, 1.0), 0.0);

    aExtra = 1.70;   % [m/s^2]

    Fx_cmd = -m * aExtra * ctrlState.absScale;
    Fx_cmd = max(Fx_cmd, -maxAddBrakeForce);

    if slipAbsMax > 0.25
        Fx_cmd = 0;
        ctrlState.absScale = 0.0;
    end

else
    Fx_cmd = 0;
    ctrlState.absScale = min(ctrlState.absScale + 0.01, 0.35);
end

%% 6. Jerk Limit 적용
maxDeltaF_apply   = m * maxJerk * dtSafe;
maxDeltaF_release = 3.0 * maxDeltaF_apply;

dF = Fx_cmd - ctrlState.prevForce;

if Fx_cmd > ctrlState.prevForce
    maxDeltaF = maxDeltaF_release;
else
    maxDeltaF = maxDeltaF_apply;
end

dF = max(min(dF, maxDeltaF), -maxDeltaF);
Fx_limited = ctrlState.prevForce + dF;

%% 7. 최종 제동력 제한
if ~isfinite(Fx_limited)
    Fx_limited = 0;
end

Fx_limited = min(Fx_limited, 0);
Fx_limited = max(Fx_limited, -maxAddBrakeForce);

ctrlState.prevForce = Fx_limited;
ctrlState.intError = 0;

%% 8. ABS Wheel별 Brake Reduction
absReduction = zeros(4,1);

if brakingLike && vxSafe > 3.0
    for i = 1:4
        s = slipAbs(i);

        if s > 0.22
            absReduction(i) = 1000;
        elseif s > 0.18
            absReduction(i) = 800;
        elseif s > 0.14
            absReduction(i) = 560;
        elseif s > 0.11
            absReduction(i) = 300;
        else
            absReduction(i) = 0;
        end
    end
end

forceCmd.absReduction = absReduction;

%% 9. 최종 종방향 제어 명령 출력
forceCmd.Fx_total = Fx_limited;

if Fx_limited < 0
    forceCmd.brakeRatio = min(max(-Fx_limited / maxAddBrakeForce, 0), 1);
else
    forceCmd.brakeRatio = 0;

end