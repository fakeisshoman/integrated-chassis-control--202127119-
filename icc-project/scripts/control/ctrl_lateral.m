function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC) — 학번 202127119
%
%   설계 개요 (v6 — yaw-error 게이팅 ESC):
%     - AFS: yaw rate 추종 PID 보정 (소권한, ±5deg). Ki 는 slip angle 스케줄링.
%     - ESC(beta-limiter): |β|>임계 시 yaw moment. 단, "yaw rate 추종오차"가
%       작은 상황(정상 선회 중 제동, 예 A7 brake-in-turn)에서만 작동하도록
%       게이팅한다. DLC(A1)처럼 yaw 가 좌우로 격렬히 진동해 순간 추종오차가
%       큰 회피기동에서는 ESC 차동 brake 가 오히려 차체를 교란(sideSlip↑)하므로
%       ESC 를 비활성화 → 베이스라인(off) 대비 악화를 방지한다.
%       게이트는 yawErr 의 저역통과(평활) 값으로 판정해 chattering 을 막는다.
%
%   Inputs:  yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt
%   Outputs: deltaAdd.steerAngle [rad], deltaAdd.yawMoment [Nm], ctrlState

    %% ---- 0. 상태 초기화 ----
    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0;  end
    if ~isfield(ctrlState, 'prevDelta'); ctrlState.prevDelta = 0;  end
    if ~isfield(ctrlState, 'prevErr');   ctrlState.prevErr   = 0;  end
    if ~isfield(ctrlState, 'yawErrLP');  ctrlState.yawErrLP  = 0;  end

    %% ---- 1. ref 물리 클램프 ----
    mu_g   = 9.81 * 1.0;
    vx_s   = max(abs(vx), 1.0);
    r_phys = mu_g / vx_s;
    yawRateRef = max(-r_phys, min(r_phys, yawRateRef));

    %% ---- 2. AFS: 소권한 yaw 보정 (PID) ----
    yawErr = yawRateRef - yawRate;

    Kp_corr = 0.10;
    Kd_corr = 0.030;
    beta_abs = abs(slipAngle);
    b_lo = deg2rad(1.5);   b_hi = deg2rad(4.0);
    Ki_lo = 0.01;          Ki_hi = 0.10;
    if beta_abs <= b_lo
        Ki_corr = Ki_lo;
    elseif beta_abs >= b_hi
        Ki_corr = Ki_hi;
    else
        Ki_corr = Ki_lo + (Ki_hi - Ki_lo) * (beta_abs - b_lo) / (b_hi - b_lo);
    end
    intMax  = 0.10;

    ctrlState.intError = ctrlState.intError + yawErr * dt;
    ctrlState.intError = max(-intMax, min(intMax, ctrlState.intError));

    dErr = (yawErr - ctrlState.prevErr) / max(dt, 1e-4);
    ctrlState.prevErr = yawErr;

    delta_AFS = Kp_corr * yawErr + Ki_corr * ctrlState.intError + Kd_corr * dErr;

    fv = min(max(20.0 / vx_s, 0.5), 1.5);
    delta_AFS = delta_AFS * fv;

    %% ---- 3. AFS 권한 제한 + saturation + rate limit ----
    AFS_AUTH = deg2rad(5.0);
    delta_AFS = max(-AFS_AUTH, min(AFS_AUTH, delta_AFS));
    delta_AFS = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, delta_AFS));
    if isfield(LIM, 'MAX_STEER_RATE')
        dmax = LIM.MAX_STEER_RATE * dt;
        delta_AFS = max(ctrlState.prevDelta - dmax, ...
                        min(ctrlState.prevDelta + dmax, delta_AFS));
    end
    ctrlState.prevDelta = delta_AFS;

    %% ---- 4. ESC: yaw-error 게이팅 beta-limiter ----
    %  yawErr 저역통과(평활): 순간 진동 제거. tau≈0.15s
    alpha = dt / (0.15 + dt);
    ctrlState.yawErrLP = (1-alpha)*ctrlState.yawErrLP + alpha*abs(yawErr);
    yawErrSmooth = ctrlState.yawErrLP;

    % 게이트: 추종오차가 작을 때(정상선회 제동, A7)만 ESC full,
    %         크면(DLC 회피, A1) ESC 비활성. 0.2~0.5 rad/s 사이 선형 전이.
    g_lo = 0.20;  g_hi = 0.50;
    if yawErrSmooth <= g_lo
        esc_gate = 1.0;
    elseif yawErrSmooth >= g_hi
        esc_gate = 0.0;
    else
        esc_gate = 1.0 - (yawErrSmooth - g_lo) / (g_hi - g_lo);
    end

    beta_th = deg2rad(3.0);
    if isfield(LIM, 'MAX_SLIP_ANGLE')
        beta_th = min(beta_th, 0.6 * LIM.MAX_SLIP_ANGLE);
    end
    K_beta  = 1.5e4;
    Mz_max  = 3000;

    yawMoment = 0;
    if abs(slipAngle) > beta_th
        fvb    = min(max(vx_s / 20.0, 1.0), 1.5);
        excess = abs(slipAngle) - beta_th;
        yawMoment = -K_beta * sign(slipAngle) * excess * fvb;
        yawMoment = max(-Mz_max, min(Mz_max, yawMoment));
        yawMoment = yawMoment * esc_gate;     % yaw-error 게이팅 적용
    end

    %% ---- 5. 출력 ----
    deltaAdd.steerAngle = delta_AFS;
    deltaAdd.yawMoment  = yawMoment;
end
