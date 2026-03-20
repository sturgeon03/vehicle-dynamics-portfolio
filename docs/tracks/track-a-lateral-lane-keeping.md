# Track A - Lateral Dynamics and Lane Keeping

Track A는 lateral vehicle dynamics와 automated lane keeping을 하나의 이야기로 묶는다.

현재 공개 범위에서 Chapter 3는 3.6까지 열어 두고, full-state lane-keeping baseline과 곡률 feedforward, steady-state cornering 해석, varying-speed stability intuition, look-ahead output measurement, 그리고 unity feedback loop를 Track A 흐름에 붙인다.

## Chapters

- Chapter 2. Lateral Vehicle Dynamics
- Chapter 3. Steering Control for Automated Lane Keeping

## Signature Artifacts

- bicycle model 상태 변수 설명
- yaw rate / slip angle 직관 해설
- road-relative error model에서 global trajectory까지 이어지는 state bridge
- clothoid road model과 lane keeping reference 해설
- state feedback와 curvature feedforward 비교 노트
- steady-state steering angle과 understeer / neutral steer / oversteer 해설
- varying longitudinal velocity에서의 constant-gain stability 해설
- look-ahead lateral position measurement와 output-feedback plant 해설
- sensor location에 따른 open-loop zero damping 비교
