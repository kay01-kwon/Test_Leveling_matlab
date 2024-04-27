function transform = SE3_transform(Rotm,t)
%SE3 이 함수의 요약 설명 위치
%   자세한 설명 위치

transform = [Rotm t;
    zeros(1,3) 1];

end