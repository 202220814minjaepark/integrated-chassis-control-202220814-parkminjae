function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    %  (1) lonCmd.Fx_total → 4-wheel 균등 brake (with 60:40 split)
    %  (2) latCmd.yawMoment → 4-wheel 차동 brake
    %  (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
    %  (4) verCmd → actuatorCmd.dampingCoeff (pass-through 또는 추가 가공)
    %  (5) 최종 saturation
%% 1. 기본 파라미터 설정
r_w = 0.33;      % tire effective radius [m]
t_f = 1.56;      % front track width [m]
t_r = 1.52;      % rear track width [m]

ratio_f = 0.60;
ratio_r = 1.0 - ratio_f;

brakeTorque = zeros(4, 1);   % [FL; FR; RL; RR]

%% 2. 제한값 예외 처리
if nargin < 7 || isempty(LIM) || ~isstruct(LIM)
    LIM = struct();
end

if isfield(LIM, 'MAX_BRAKE_TRQ') && isfinite(LIM.MAX_BRAKE_TRQ) && LIM.MAX_BRAKE_TRQ > 0
    maxBrakeTrq = LIM.MAX_BRAKE_TRQ;
else
    maxBrakeTrq = 4000;
end

if isfield(LIM, 'MAX_STEER_ANGLE') && isfinite(LIM.MAX_STEER_ANGLE) && LIM.MAX_STEER_ANGLE > 0
    maxSteer = LIM.MAX_STEER_ANGLE;
else
    maxSteer = 0.5;
end

%% 3. 입력 명령 예외 처리
if nargin < 1 || isempty(latCmd) || ~isstruct(latCmd)
    latCmd = struct();
end

if nargin < 2 || isempty(lonCmd) || ~isstruct(lonCmd)
    lonCmd = struct();
end

if nargin < 3
    verCmd = [];
end

if ~isfield(latCmd, 'steerAngle')
    latCmd.steerAngle = 0;
end

if ~isfield(latCmd, 'yawMoment')
    latCmd.yawMoment = 0;
end

if ~isfield(lonCmd, 'Fx_total')
    lonCmd.Fx_total = 0;
end

if ~isfield(lonCmd, 'brakeRatio')
    lonCmd.brakeRatio = 0;
end

if isempty(latCmd.steerAngle) || ~isscalar(latCmd.steerAngle) || ~isfinite(latCmd.steerAngle)
    latCmd.steerAngle = 0;
end

if isempty(latCmd.yawMoment) || ~isscalar(latCmd.yawMoment) || ~isfinite(latCmd.yawMoment)
    latCmd.yawMoment = 0;
end

if isempty(lonCmd.Fx_total) || ~isscalar(lonCmd.Fx_total) || ~isfinite(lonCmd.Fx_total)
    lonCmd.Fx_total = 0;
end

if isempty(lonCmd.brakeRatio) || ~isscalar(lonCmd.brakeRatio) || ~isfinite(lonCmd.brakeRatio)
    lonCmd.brakeRatio = 0;
end

%% 4. AFS 조향각 제한
steer = latCmd.steerAngle;
steer = max(min(steer, maxSteer), -maxSteer);

%% 5. 종방향 제동력 분배
Fx = lonCmd.Fx_total;
Mz = latCmd.yawMoment;

straightBrakeOK = abs(steer) < 0.0005 && abs(Mz) < 2.0;

if straightBrakeOK && Fx < 0

    F_brake = abs(Fx);

    if ~isfinite(F_brake)
        F_brake = 0;
    end

    F_brake_max = 4 * maxBrakeTrq / r_w;
    F_brake = max(min(F_brake, F_brake_max), 0);

    T_front_each = (ratio_f * F_brake / 2) * r_w;
    T_rear_each  = (ratio_r * F_brake / 2) * r_w;

    brakeTorque(1) = brakeTorque(1) + T_front_each;   % FL
    brakeTorque(2) = brakeTorque(2) + T_front_each;   % FR
    brakeTorque(3) = brakeTorque(3) + T_rear_each;    % RL
    brakeTorque(4) = brakeTorque(4) + T_rear_each;    % RR
end

%% 6. ABS Release 보정
if straightBrakeOK && isfield(lonCmd, 'absReduction')

    absReduction = lonCmd.absReduction(:);

    if numel(absReduction) < 4
        absReduction = [absReduction; zeros(4 - numel(absReduction), 1)];
    elseif numel(absReduction) > 4
        absReduction = absReduction(1:4);
    end

    absReduction(~isfinite(absReduction)) = 0;
    absReduction = max(min(absReduction, 1200), 0);

    brakeTorque = brakeTorque - absReduction;
end

%% 7. ESC Yaw Moment 차동 제동 분배
escGain = 0.20;

if abs(Mz) > 1.0 && escGain > 0

    escRatioF = 0.45;
    escRatioR = 0.55;

    dT_f = escGain * (abs(Mz) * escRatioF / t_f) * r_w;
    dT_r = escGain * (abs(Mz) * escRatioR / t_r) * r_w;

    if ~isfinite(dT_f)
        dT_f = 0;
    end

    if ~isfinite(dT_r)
        dT_r = 0;
    end

    if Mz > 0
        brakeTorque(1) = brakeTorque(1) + dT_f;   % FL
        brakeTorque(3) = brakeTorque(3) + dT_r;   % RL
    else
        brakeTorque(2) = brakeTorque(2) + dT_f;   % FR
        brakeTorque(4) = brakeTorque(4) + dT_r;   % RR
    end
end

%% 8. Brake Torque 제한
brakeTorque(~isfinite(brakeTorque)) = 0;
brakeTorque = min(brakeTorque, maxBrakeTrq);

%% 9. 수직 감쇠 명령 전달
if isempty(verCmd)
    dampCoeff = zeros(4, 1);

elseif isstruct(verCmd)
    if isfield(verCmd, 'dampingCoeff')
        dampCoeff = verCmd.dampingCoeff(:);
    else
        dampCoeff = zeros(4, 1);
    end

else
    dampCoeff = verCmd(:);
end

if numel(dampCoeff) < 4
    dampCoeff = [dampCoeff; zeros(4 - numel(dampCoeff), 1)];
elseif numel(dampCoeff) > 4
    dampCoeff = dampCoeff(1:4);
end

dampCoeff(~isfinite(dampCoeff)) = 0;

%% 10. 최종 actuator 명령 출력
actuatorCmd.steerAngle   = steer;
actuatorCmd.brakeTorque  = brakeTorque;
actuatorCmd.dampingCoeff = dampCoeff;

end