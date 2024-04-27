function Rotm = Rotm_LIFT(q_lift)
%ROTM_LIFT 이 함수의 요약 설명 위치
%   자세한 설명 위치
cq = cos(q_lift);
sq = sin(q_lift);

Rotm = [cq 0 -sq;
    0 1 0;
    sq 0 cq];
end

