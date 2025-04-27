function [M, CG, I_CG] = getAssemblyProps(cList)
%-------------------------------------------------------------
%   여러 컴포넌트 질량·CG·MOI 집계 (평행축 정리)
%-------------------------------------------------------------
if iscell(cList),  cList = [cList{:}];  end
if isempty(cList), M=0; CG=[0 0 0]; I_CG=zeros(3); return; end

N        = numel(cList);
m        = zeros(N,1);            % 질량 열-벡터
cgMatrix = zeros(N,3);            % CG 좌표 N×3

for k = 1:N
    % ---------- 질량 ----------
    mass_k = cList(k).mass;
    if isempty(mass_k) || ~isscalar(mass_k) || isnan(mass_k)
        mass_k = 0;               % 무효 → 0 kg
    end
    m(k) = mass_k;

    % ---------- CG ----------
    cg_k = reshape(cList(k).cg_ref,1,[]);   % 행 벡터화
    if numel(cg_k)~=3 || any(isnan(cg_k))
        cg_k = [0 0 0];           % 무효 → 원점
    end
    cgMatrix(k,:) = cg_k;
end

M  = sum(m);
    if M == 0,  CG=[0 0 0]; I_CG=zeros(3); return; end
    
    CG = (m.' * cgMatrix) / M;        % 1×3
    
    % ---------- 평행축 정리 ----------
    I_CG = zeros(3);      I3 = eye(3);
        for k = 1:N
            if m(k)==0,  continue;  end    % 질량 0 → 건너뜀
            r  = cgMatrix(k,:) - CG;       % 1×3
            d2 = r * r.';                  % |r|²
            I_CG = I_CG ...
                  + cList(k).inertia_cg ...
                  + m(k) * (d2*I3 - (r.' * r));
        end
end
