function Rotm = Rotm_BODY(pitch,roll)
%ROTM_BODY 이 함수의 요약 설명 위치
%   자세한 설명 위치

cp = cos(pitch);
sp = sin(pitch);

cr = cos(roll);
sr = sin(roll);

Rotm_p = [cp 0 sp;
    0 1 0;
    -sp 0 cp];

Rotm_r = [1 0 0;
    0 cr -sr;
    0 sr cr];

Rotm = Rotm_p*Rotm_r;
end

