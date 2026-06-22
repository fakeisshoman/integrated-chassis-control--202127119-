function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL 횡방향 통합 제어기 (AFS + ESC) — 학번 202127119
%
%   설계 개요 (v2 — 보정형 AFS):
%     driver(Stanley)가 이미 경로추종을 수행하므로, AFS 는 yaw rate 추종오차를
%     메우는 "소권한 보정기"로 동작한다. 전체 조향을 다시 만들지 않는다.
%     - AFS: yaw rate 오차에 대한 PI 보정 + 조향 권한 제한(AFS_AUTH ±5deg).
%     - ref 클램프: calc_ref_yaw_rate 가 과도구간에서 물리한계(타이어 포화)를
%       넘는 목표를 줄 수 있어, ay 기준 최대 yaw rate(mu*g/vx)로 제한.
%     - ESC(beta-limiter): |beta|>임계 시 보수적 yaw moment.
%
%   Inputs:  yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt
%   Outputs: deltaAdd.steerAngle [rad], deltaAdd.yawMoment [Nm], ctrlState

    %% ---- 0. 상태 초기화 ----
    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0;  end
    if ~isfield(ctrlState, 'prevDelta'); ctrlState.prevDelta = 0;  end
    if ~isfield(ctrlState, 'prevErr');   ctrlState.prevErr   = 0;  end

    %% ---- 1. ref 물리 클램프 ----
    mu_g   = 9.81 * 1.0;
    vx_s   = max(abs(vx), 1.0);
    r_phys = mu_g / vx_s;
    yawRateRef = max(-r_phys, min(r_phys, yawRateRef));

    %% ---- 2. AFS: 소권한 yaw 보정 (PID) ----
    yawErr = yawRateRef - yawRate;

    % 보정 게인 — P/I/D. D항이 과도구간 overshoot 를 댐핑한다.
    Kp_corr = 0.08;
    Ki_corr = 0.06;    % 적분 축소 (step 응답 위상지연 완화)
    Kd_corr = 0.015;   % 미분 추가 (overshoot 억제)
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

    %% ---- 4. ESC: beta-limiter (보수적) ----
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
    end

    %% ---- 5. 출력 ----
    deltaAdd.steerAngle = delta_AFS;
    deltaAdd.yawMoment  = yawMoment;
end
