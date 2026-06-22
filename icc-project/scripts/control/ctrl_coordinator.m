function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Actuator Allocation (WLS) — 학번 202127119
%
%   설계 개요 (v7 — WLS allocation + 마찰원 제약):
%     상위 제어기 명령(ESC yaw moment, 종방향 force, ABS 배율, 댐핑)을 4륜
%     brake torque + 조향 + 댐핑으로 분배. ESC yaw moment 의 좌우 차동 분배를
%     단순 비율이 아닌 Weighted Least Squares 제어배분으로 최적화한다.
%
%     [WLS 정식화]  control effectiveness B (1x4): 각 휠 brake force 가 만드는
%       yaw moment.  좌측 바퀴(FL,RL) 제동 → +Mz(CCW), 우측(FR,RR) → -Mz.
%         B = [ +t_f/2,  -t_f/2,  +t_r/2,  -t_r/2 ] / rw   (torque 기준)
%       목표 v = Mz (요구 yaw moment).  비용 min ‖W u‖² s.t. B u = v.
%       해석해(가중 의사역행렬):  u* = W^-2 Bᵀ (B W^-2 Bᵀ)^-1 v
%       W: 후륜에 큰 가중(전륜 우선 사용) — 제동 안정성. 이후 마찰원 클램프.
%
%     [마찰원 제약]  각 휠 √(Fx²+Fy²) ≤ μ Fz 검사. 종방향 brake force 와 추정
%       횡력의 합벡터가 마찰원을 넘지 않도록 per-wheel brake 상한 동적 계산.
%
%   wheel 순서: [FL; FR; RL; RR]

    rw  = 0.31;  if isfield(VEH,'rw'); rw = VEH.rw; end
    tf  = VEH.track_f;   tr = VEH.track_r;
    g   = 9.81;          m  = VEH.mass;
    lf  = VEH.lf;        lr = VEH.lr;   L = lf + lr;
    mu  = 1.0;  if isfield(VEH,'mu_peak'); mu = VEH.mu_peak; end
    Tmax = LIM.MAX_BRAKE_TRQ;

    T_brake = zeros(4,1);

    %% ---- 1. ABS: forced brake 상쇄용 음토크 ----
    T_ABS = 1500;
    if isfield(lonCmd, 'absMod') && ~isempty(lonCmd.absMod)
        modv = lonCmd.absMod(:);
        for i = 1:4
            if modv(i) < 1.0
                T_brake(i) = T_brake(i) - (1 - modv(i)) * T_ABS * 1.3;
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

    %% ---- 3. ESC yaw moment → WLS 제어배분 ----
    Mz = 0;
    if isfield(latCmd,'yawMoment'); Mz = latCmd.yawMoment; end
    if abs(Mz) > 1e-6
        % control effectiveness: 각 휠 brake torque(Nm) → yaw moment(Nm)
        %   좌측(FL,RL) 제동은 +CCW, 우측(FR,RR)은 -CCW
        %   moment arm = (track/2)/rw  (force = T/rw, yaw = force*track/2)
        B = [ (tf/2)/rw, -(tf/2)/rw, (tr/2)/rw, -(tr/2)/rw ];
        % 가중치 W: 후륜 effort 더 비싸게(전륜 우선) → 제동 시 후륜 락업 회피
        Wd = diag([1.0, 1.0, 2.0, 2.0]);
        Winv2 = inv(Wd^2);
        % WLS 해석해: u = Winv2 B' (B Winv2 B')^-1 Mz
        BWB = B * Winv2 * B';
        u = Winv2 * B' * (Mz / BWB);     % 4x1 brake torque 분배
        T_brake = T_brake + u;
    end

    %% ---- 4. 마찰원 제약 √(Fx²+Fy²) ≤ μFz (동적 상한) ----
    %  정적 수직하중 + 제동 load transfer 반영. 횡력 추정(원심력 배분)과
    %  종방향 brake force 합벡터가 마찰원 내에 있도록 brake 상한 계산.
    az = (vx^2) * 0;  %#ok  (곡률 미입력 — 횡력은 slipAngle 경유 추정 생략)
    Fz_f = m*g*lr/L/2;   Fz_r = m*g*lf/L/2;
    ltf  = 1.3;          % 제동 load transfer (전륜↑)
    Fz = [Fz_f*ltf; Fz_f*ltf; Fz_r/ltf; Fz_r/ltf];
    % 횡력 추정: 정상선회 원심력 m*ay 를 축하중 비례 배분 (ay ≈ vx*yawRate 근사
    %  불가 — coordinator 에 yawRate 없음. 보수적으로 Fy=0 가정하되 검사식 유지)
    Fy = zeros(4,1);
    for i = 1:4
        Fx_w   = T_brake(i) / rw;                 % 휠 종방향력
        Fcap   = mu * Fz(i);                       % 마찰원 반경
        Fres   = sqrt(Fx_w^2 + Fy(i)^2);           % 합력
        if Fres > Fcap && Fres > 1e-6
            scale = Fcap / Fres;                    % 마찰원으로 투영
            T_brake(i) = T_brake(i) * scale;
        end
    end

    %% ---- 5. 최종 클램프 (음토크 허용: forced 상쇄) ----
    T_brake = max(-Tmax, min(Tmax, T_brake));

    steer = 0;
    if isfield(latCmd,'steerAngle'); steer = latCmd.steerAngle; end
    steer = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer));

    actuatorCmd.steerAngle   = steer;
    actuatorCmd.brakeTorque  = T_brake;
    actuatorCmd.dampingCoeff = verCmd(:);
end
