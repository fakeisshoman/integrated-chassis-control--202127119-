function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL CDC (Continuous Damping Control) — 학번 202127119
%
%   설계 개요:
%     - Skyhook semi-active 댐핑. sprung mass 의 절대 수직속도를 줄이는 방향으로
%       per-wheel 감쇠계수를 연속 변조.
%     - 판정:  zs_dot * (zs_dot - zu_dot) > 0  → 댐퍼가 sprung 안정화에 기여 →
%       감쇠 강화(cMax 쪽). 그 외 구간은 cMin 으로 낮춰 노면 격리(ride) 개선.
%     - 연속형(continuous): on-off 의 chattering 을 피하려 skyGain 으로 비례 변조
%       후 [cMin,cMax] 클램프.
%
%   Inputs:
%       suspState - .zs_dot(4), .zu_dot(4), .zs(4), .zu(4)
%       ctrlState - 내부 상태 (미사용, 확장 여지)
%       CTRL      - .VER.cMin, .cMax, .skyGain
%       dt        - sample time
%
%   Output:
%       dampingCmd - 4x1 damping [Ns/m]

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    skyG = CTRL.VER.skyGain;

    % suspState 가 비어있으면(저차원 plant) passive 기본값
    if ~isfield(suspState, 'zs_dot') || isempty(suspState.zs_dot)
        dampingCmd = 0.5 * (cMin + cMax) * ones(4,1);
        return;
    end

    zs_dot = suspState.zs_dot(:);
    if isfield(suspState, 'zu_dot') && ~isempty(suspState.zu_dot)
        zu_dot = suspState.zu_dot(:);
    else
        zu_dot = zeros(4,1);
    end

    vrel = zs_dot - zu_dot;            % 댐퍼 양단 상대속도
    dampingCmd = zeros(4,1);

    for i = 1:4
        if zs_dot(i) * vrel(i) > 0
            % skyhook 활성: 절대속도에 비례해 감쇠 강화
            if abs(vrel(i)) > 1e-4
                c = skyG * zs_dot(i) / vrel(i);
            else
                c = cMax;
            end
        else
            % 댐퍼가 sprung 을 가진(逆효과) 구간 → 최소 감쇠
            c = cMin;
        end
        dampingCmd(i) = max(cMin, min(cMax, c));
    end
end
