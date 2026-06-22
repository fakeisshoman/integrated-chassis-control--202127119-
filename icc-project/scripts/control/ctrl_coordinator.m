function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation — 학번 202127119
%
%   설계 개요 (v2 — ABS 음토크 + ESC 차동 + 마찰원 제한):
%     runner 구조상 시나리오 forced brake 는 coordinator 를 거치지 않고
%     brake_total = brk_scenario + brakeTorque 로 합산된 뒤 max(0,·) 클리핑된다.
%       - ABS: lonCmd.absMod(4x1, 0~1)이 1 미만인 휠은 음토크로 forced 를 깎아
%         락업 방지 (absMod=0 → 완전 해제).
%       - ESC: latCmd.yawMoment 를 좌우 차동 brake(양수)로 환산.
%       - 마찰원 제한: per-wheel 양토크 상한 mu*Fz*rw.
%
%   wheel 순서: [FL; FR; RL; RR]

    rw  = 0.31;  if isfield(VEH,'rw'); rw = VEH.rw; end
    tf  = VEH.track_f;   tr = VEH.track_r;
    g   = 9.81;          m  = VEH.mass;
    Tmax = LIM.MAX_BRAKE_TRQ;

    T_brake = zeros(4,1);

    %% ---- 1. ABS: forced brake 상쇄용 음토크 ----
    T_ABS = 1500;
    if isfield(lonCmd, 'absMod') && ~isempty(lonCmd.absMod)
        mod = lonCmd.absMod(:);
        for i = 1:4
            if mod(i) < 1.0
                T_brake(i) = T_brake(i) - (1 - mod(i)) * T_ABS * 1.3;
            end
        end
    end

    %% ---- 2. 종방향 추가 제동 (controller 요구 시) ----
    Fx = 0;
    if isfield(lonCmd,'Fx_total'); Fx = lonCmd.Fx_total; end
    if Fx < 0
        Fb = abs(Fx);
        T_brake(1) = T_brake(1) + 0.5*0.6*Fb*rw;
        T_brake(2) = T_brake(2) + 0.5*0.6*Fb*rw;
        T_brake(3) = T_brake(3) + 0.5*0.4*Fb*rw;
        T_brake(4) = T_brake(4) + 0.5*0.4*Fb*rw;
    end

    %% ---- 3. ESC yaw moment → 좌우 차동 ----
    Mz = 0;
    if isfield(latCmd,'yawMoment'); Mz = latCmd.yawMoment; end
    ratio_f = 0.6;
    dT_f = abs(Mz) * ratio_f       / (tf/2);
    dT_r = abs(Mz) * (1 - ratio_f) / (tr/2);
    half_f = 0.5 * dT_f;  half_r = 0.5 * dT_r;
    if Mz >= 0
        T_brake(2) = T_brake(2) + half_f;   % FR
        T_brake(4) = T_brake(4) + half_r;   % RR
    else
        T_brake(1) = T_brake(1) + half_f;   % FL
        T_brake(3) = T_brake(3) + half_r;   % RL
    end

    %% ---- 4. 마찰원 제한 (양토크 상한) ----
    mu = 1.0;  if isfield(VEH,'mu_peak'); mu = VEH.mu_peak; end
    lf = VEH.lf; lr = VEH.lr; L = lf + lr;
    Tmax_f = mu * (m*g*lr/L/2) * rw;
    Tmax_r = mu * (m*g*lf/L/2) * rw;
    ub = [Tmax_f; Tmax_f; Tmax_r; Tmax_r];
    for i = 1:4
        if T_brake(i) > 0
            T_brake(i) = min(T_brake(i), ub(i));
        end
    end

    %% ---- 5. 최종 클램프 (음토크 허용) ----
    T_brake = max(-Tmax, min(Tmax, T_brake));

    steer = 0;
    if isfield(latCmd,'steerAngle'); steer = latCmd.steerAngle; end
    steer = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer));

    actuatorCmd.steerAngle   = steer;
    actuatorCmd.brakeTorque  = T_brake;
    actuatorCmd.dampingCoeff = verCmd(:);
end
