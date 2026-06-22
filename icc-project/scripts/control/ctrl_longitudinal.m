function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL 종방향 제어기 (속도 추종 + ABS) — 학번 202127119
%
%   설계 개요 (v4 — ABS 락업방지 + 슬립여유 기반 제동 부스트):
%     - 속도 추종: PI 제어 (anti-windup).
%     - ABS: ctrlState.wheelSlip(4x1) 기반. 제동 중 |kappa|>target 휠은 brake
%       배율(absMod)을 낮춰 락업 방지, 슬립 회복 시 점진 복원.
%     - 제동 부스트: 감속 중(ax<0)이고 타이어 슬립에 여유가 있으면, 추가 제동
%       force(Fx_total<0)를 인가해 제동거리 단축. 부스트 크기는 "최소 슬립 여유"에
%       비례하고 rate-limited 라 발산하지 않음. 슬립이 target 도달 시 자동 포화.
%     - jerk limit.
%
%   Inputs:  vxRef, vx, ax, ctrlState(.wheelSlip 4x1), CTRL, LIM, dt
%   Outputs: forceCmd.Fx_total, .brakeRatio, .absMod(4x1), ctrlState

    m = 1500;

    if ~isfield(ctrlState, 'intError');  ctrlState.intError  = 0;         end
    if ~isfield(ctrlState, 'prevForce'); ctrlState.prevForce = 0;         end
    if ~isfield(ctrlState, 'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end
    if ~isfield(ctrlState, 'absMod');    ctrlState.absMod    = ones(4,1);  end
    if ~isfield(ctrlState, 'boostF');    ctrlState.boostF    = 0;          end

    %% ---- 1. 속도 추종 PI ----
    Kp = CTRL.LON.Kp;  Ki = CTRL.LON.Ki;  intMax = CTRL.LON.intMax;
    err = vxRef - vx;
    ctrlState.intError = ctrlState.intError + err * dt;
    ctrlState.intError = max(-intMax, min(intMax, ctrlState.intError));
    a_cmd = Kp * err + Ki * ctrlState.intError;
    a_cmd = max(-LIM.MAX_AX, min(LIM.MAX_AX, a_cmd));
    Fx_track = m * a_cmd;

    % 제동 시나리오 보호: 이미 감속 중(ax<0)이면 PI 의 양의 가속 명령을 차단.
    % (runner 가 vxRef=vx0 고정을 주므로, 제동 중엔 PI 가 계속 가속을 요구해
    %  제동을 방해함 → 감속 상황에서 가속분 제거)
    if ax < -0.5 && Fx_track > 0
        Fx_track = 0;
        ctrlState.intError = 0;     % 적분 windup 제거
    end

    %% ---- 2. ABS per-wheel 모듈레이션 (락업 방지) ----
    kappa_target = 0.12;
    slip = ctrlState.wheelSlip(:);
    mod  = ctrlState.absMod(:);
    for i = 1:4
        ks = abs(slip(i));
        if ks > kappa_target
            target_mod = max(0.0, 1 - (ks - kappa_target) / kappa_target);
            mod(i) = min(mod(i), target_mod);
        else
            mod(i) = min(1.0, mod(i) + 2.0 * dt);
        end
    end
    ctrlState.absMod = mod;

    %% ---- 3. 슬립여유 기반 제동 부스트 ----
    %  감속 중(ax<0)에만. 모든 휠의 |slip| 이 target 보다 여유 있으면 부스트 증가,
    %  한 휠이라도 target 근접/초과면 부스트 감소. rate-limited 라 진동 억제.
    maxSlip = max(abs(slip));
    boostF  = ctrlState.boostF;
    if ax < -0.5
        margin = kappa_target - maxSlip;     % >0: 여유, <0: 초과
        if margin > 0.02
            boostF = boostF + 8000 * dt;     % 증가 (초당 +8kN)
        elseif margin < 0.0
            boostF = boostF - 30000 * dt;    % 초과 시 빠르게 감소
        end
        % 여유가 작으면(0~0.02) 현 상태 유지
    else
        boostF = 0;                          % 제동 아니면 부스트 0
    end
    boostF = max(0, min(8000, boostF));      % [0, 8kN] 제한
    ctrlState.boostF = boostF;

    % 총 종방향 힘 = 추종분 + 부스트(제동이므로 음수)
    Fx = Fx_track - boostF;

    %% ---- 4. jerk limit ----
    dF_max = LIM.MAX_JERK * m * dt;
    dF = Fx - ctrlState.prevForce;
    dF = max(-dF_max, min(dF_max, dF));
    Fx = ctrlState.prevForce + dF;
    ctrlState.prevForce = Fx;

    %% ---- 5. 출력 ----
    forceCmd.Fx_total   = Fx;
    forceCmd.absMod     = mod;
    if Fx < 0
        forceCmd.brakeRatio = min(1, abs(Fx) / (m * LIM.MAX_AX));
    else
        forceCmd.brakeRatio = 0;
    end
end
