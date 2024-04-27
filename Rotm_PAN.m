function Rotm = Rotm_PAN(q_pan)
%ROTM_PAN 이 함수의 요약 설명 위치
%   자세한 설명 위치

cq = cos(q_pan);
sq = sin(q_pan);
Rotm = [cq sq 0;
    -sq cq 0;
    0 0 1];

end

