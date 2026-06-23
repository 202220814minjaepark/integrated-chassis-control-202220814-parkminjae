function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% TODO: 여기에 학생 구현 작성
    %  (1) PID/LQR/... 으로 yaw rate 추종 보조 조향 계산
    %  (2) slip angle 임계 초과 시 yaw moment 계산
    %  (3) speed scheduling 적용
    %  (4) limit/saturation
% 1. 입력 예외 처리 및 내부 상태 초기화
if nargin < 8 || isempty(dt) || dt <= 0
    dt = 0.001;
end
dtSafe = max(dt, 1e-6);

if isempty(ctrlState) || ~isstruct(ctrlState)
    ctrlState = struct();
end
if ~isfield(ctrlState, 'intError'),  ctrlState.intError  = 0; end
if ~isfield(ctrlState, 'prevError'), ctrlState.prevError = 0; end
if ~isfield(ctrlState, 'prevDeriv'), ctrlState.prevDeriv = 0; end
if ~isfield(ctrlState, 'prevSteer'), ctrlState.prevSteer = 0; end

deltaAdd.steerAngle = 0;
deltaAdd.yawMoment  = 0;

vxSafe = max(abs(vx), 0.5);

if vxSafe < 2.0
    ctrlState.intError  = 0;
    ctrlState.prevError = 0;
    ctrlState.prevDeriv = 0;
    ctrlState.prevSteer = 0;
    return;
end

% 2. Yaw Rate 오차 기반 AFS 제어
yaw_error = yawRateRef - yawRate;

if ~isfinite(yaw_error)
    yaw_error = 0;
end

yaw_error = max(min(yaw_error, 2.0), -2.0);

intMax = 0.5;
if isfield(CTRL.LAT, 'intMax')
    intMax = min(CTRL.LAT.intMax, 0.5);
end

ctrlState.intError = ctrlState.intError + yaw_error * dtSafe;
ctrlState.intError = max(min(ctrlState.intError, intMax), -intMax);

raw_deriv = (yaw_error - ctrlState.prevError) / dtSafe;

if ~isfinite(raw_deriv)
    raw_deriv = 0;
end

raw_deriv = max(min(raw_deriv, 20), -20);

alpha = 0.90;
deriv = alpha * ctrlState.prevDeriv + (1 - alpha) * raw_deriv;

ctrlState.prevError = yaw_error;
ctrlState.prevDeriv = deriv;

Kp = 0.80 * CTRL.LAT.Kp;
Ki = 0.05 * CTRL.LAT.Ki;
Kd = 1.20 * CTRL.LAT.Kd;

vRef = 18.0;
speedScale = vRef / vxSafe;
speedScale = max(min(speedScale, 1.0), 0.35);

steer_yaw = speedScale * ...
    (Kp * yaw_error + Ki * ctrlState.intError + Kd * deriv);

% 3. Slip Angle 기반 Counter-Steer 제어
beta = slipAngle;

if ~isfinite(beta)
    beta = 0;
end

betaTh = deg2rad(2.7);
steer_slip = 0;

if abs(beta) > betaTh
    betaExcess = abs(beta) - betaTh;

    KslipSteer = 0.80;
    slipSpeedScale = vxSafe / 15.0;
    slipSpeedScale = max(min(slipSpeedScale, 1.4), 0.5);

    steer_slip = -KslipSteer * sign(beta) * betaExcess * slipSpeedScale;
end

% 4. AFS 명령 합산 및 제한
steer_cmd = steer_yaw + steer_slip;

if ~isfinite(steer_cmd)
    steer_cmd = 0;
end

afs_limit = 0.14 * LIM.MAX_STEER_ANGLE;
steer_sat = max(min(steer_cmd, afs_limit), -afs_limit);

maxStep = 0.70 * dtSafe;

steer_rate_limited = ctrlState.prevSteer + ...
    max(min(steer_sat - ctrlState.prevSteer, maxStep), -maxStep);

if abs(steer_cmd) > afs_limit
    ctrlState.intError = 0.8 * ctrlState.intError;
end

if ~isfinite(steer_rate_limited)
    steer_rate_limited = 0;
end

ctrlState.prevSteer = steer_rate_limited;
deltaAdd.steerAngle = steer_rate_limited;

% 5. ESC Yaw Moment 제어
betaEscTh = deg2rad(2.35);

if abs(beta) > betaEscTh
    betaExcessEsc = abs(beta) - betaEscTh;

    Kbeta = 42000;
    escSpeedScale = vxSafe / 20.0;
    escSpeedScale = max(min(escSpeedScale, 1.2), 0.5);

    Mz_cmd = -Kbeta * sign(beta) * betaExcessEsc * escSpeedScale;
else
    Mz_cmd = 0;
end

if ~isfinite(Mz_cmd)
    Mz_cmd = 0;
end

MzMax = 4200;
deltaAdd.yawMoment = max(min(Mz_cmd, MzMax), -MzMax);
end

