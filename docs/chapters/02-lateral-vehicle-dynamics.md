# Chapter 2. Lateral Vehicle Dynamics

> 시작 페이지: p.42  
> 트랙: Track A - Lateral Dynamics and Lane Keeping

## Why This Chapter Matters

Chapter 2는 차선 유지와 요 안정성 제어를 제품 기능 소개에서 끝내지 않고, 실제 제어기 설계에 쓰는 상태공간 모델로 바꾸는 장이다. 이 장을 끝까지 따라가면 "어떤 상태를 잡아야 조향 제어가 쉬워지는가", "body-fixed 계산을 어떻게 실제 주행 궤적으로 되돌리는가", "도로 형상을 어떻게 모델링해야 추종 문제가 자연스러워지는가"가 한 흐름으로 연결된다.

## The Chapter Arc

- 2.1은 LDW, LKS, yaw stability control을 구분해 lateral control의 실제 목적을 정리한다.
- 2.2와 2.3은 저속의 kinematic bicycle model과 고속의 dynamic bicycle model을 대비해 보여 준다.
- 2.4는 `\ddot{y} + V_x r`가 왜 등장하는지 rotating frame 관점에서 설명한다.
- 2.5는 lane keeping에 직접 쓰기 좋은 road-relative error state `e_1`, `e_2`를 만든다.
- 2.6은 같은 차량을 `\beta-r` 좌표로 다시 써 yaw stability 쪽 해석을 쉽게 만든다.
- 2.7은 body-fixed 계산을 global trajectory로 복원한다.
- 2.8은 road curvature와 clothoid를 통해 "추종할 도로" 자체를 모델링한다.
- 2.9는 이 네 가지 lateral model family를 하나의 요약으로 묶는다.

## 2.1 From Product Features to Control Objectives

2.1은 lateral technology를 세 종류로 분해한다.

- LDW: 차선 이탈이 임박했음을 감지하고 경고한다.
- LKS: 조향을 직접 보조해 차선 내부를 유지한다.
- Yaw stability control: 저마찰 노면이나 과속 상황에서 nominal yaw motion에 다시 가깝게 만든다.

이 구분은 중요하다. LDW는 driver-in-the-loop 경고 시스템이고, LKS는 steering actuator가 들어간 보조 제어 시스템이며, yaw stability control은 타이어 힘과 yaw moment를 직접 다루는 안정화 문제다. Chapter 2의 모델링은 결국 이 세 문제를 같은 수학 언어로 옮기기 위한 준비 단계다.

## 2.2-2.3 Low-Speed Reference and High-Speed Plant

저속에서는 타이어 slip angle을 0으로 두는 zero-slip 가정이 충분히 유효하다고 보고, bicycle model을 이용해 기하학만으로 lateral motion을 기술할 수 있다. 핵심 결과는 global position, yaw angle, vehicle slip angle 사이의 관계를 정리하는 것이다.

$$
\dot{X} = V \cos(\beta + \psi)
$$

$$
\dot{Y} = V \sin(\beta + \psi)
$$

$$
\dot{\psi} = \frac{V \cos \beta}{l_f + l_r}\left(\tan \delta_f - \tan \delta_r\right)
$$

$$
\beta = \tan^{-1}\left(\frac{l_r \tan \delta_f + l_f \tan \delta_r}{l_f + l_r}\right)
$$

속도가 올라가면 타이어 slip angle과 횡력이 무시되지 않는다. 그래서 2.3은 lateral translation과 yaw rotation을 동시에 다루는 dynamic bicycle model로 넘어간다.

$$
m(\ddot{y} + V_x \dot{\psi}) = F_{yf} + F_{yr}
$$

$$
I_z \ddot{\psi} = l_f F_{yf} - l_r F_{yr}
$$

이 상태공간 모델이 바로 Chapter 3 steering controller 설계의 plant가 된다. 즉, 2.2가 저속 reference model이라면 2.3은 controller가 실제로 상대해야 하는 고속 dynamic model이다.

## 2.4 Why `\ddot{y} + V_x r` Appears

2.4는 앞 절 식의 핵심 coupling term이 어디서 왔는지 설명한다. body-fixed 좌표계에서 본 가속도와 inertial 좌표계에서의 가속도는 같지 않다. 회전하는 rigid body에 대해 이 둘의 관계를 쓰면 angular acceleration, centripetal, Coriolis 항이 함께 나타난다.

차량의 yaw motion만 남기는 lateral case에 적용하면 결론은 간단하다.

$$
a_y = \ddot{y} + V_x r
$$

즉, `\ddot{y}`만으로는 실제 inertial lateral acceleration이 완성되지 않는다. 차체가 yaw rotation을 하고 있기 때문에 `V_x r`가 반드시 붙는다. 이것이 2.3의 force balance를 정당화한다.

## 2.5 Road-Relative Error Model for Lane Keeping

automatic lane keeping을 목표로 하면 global pose보다 road-relative error state가 직접적이다. 그래서 2.5는 lateral dynamics를 `e_1`, `e_2` 중심의 tracking-error model로 다시 쓴다.

- `e_1`: 차량 c.g.와 차선 중심선 사이의 lateral distance
- `e_2`: 차량 yaw angle과 road tangent 사이의 orientation error

도로 반경이 `R`인 곡선도로를 일정한 longitudinal speed `V_x`로 주행한다고 가정하면

$$
\dot{\psi}_{\mathrm{des}} = \frac{V_x}{R}
$$

$$
e_2 = \psi - \psi_{\mathrm{des}}
$$

$$
\dot{e}_1 = \dot{y} + V_x e_2
$$

가 핵심 연결식이 된다. 이 정의들을 2.3의 bicycle model에 대입하면 lane keeping에 직접 쓰기 좋은 상태공간식

$$
\dot{x} = A x + B_1 \delta_f + B_2 \dot{\psi}_{\mathrm{des}}
$$

을 얻고, road bank angle까지 넣으면 disturbance input이 하나 더 붙는다. 중요한 해석은 lane keeping이 absolute pose 제어가 아니라 road-relative error stabilization 문제라는 점이다.

## 2.6 `\beta-r` Model for Lateral Stability

2.6은 같은 차량을 다른 상태 변수로 본다. 여기서는 body side slip angle `\beta`와 yaw rate `r`를 상태로 둔다.

$$
\beta \approx \frac{\dot{e}_1}{V_x} - e_2
$$

이 관계는 `e_1-e_2` 모델과 `\beta-r` 모델이 서로 다른 physics가 아니라 같은 lateral dynamics의 좌표 변환이라는 점을 보여 준다. 이 좌표계에서는

$$
m V_x (\dot{\beta} + r) = F_{yf} + F_{yr} + F_{\mathrm{bank}}
$$

$$
I_z \dot{r} = l_f F_{yf} - l_r F_{yr}
$$

처럼 쓰고, small-slip 영역에서 선형 tire force를 대입해 compact lateral plant를 얻는다. 이 표현은 lane tracking보다는 yaw stability, sideslip suppression, ESC logic을 읽을 때 더 직관적이다.

## 2.7 From Body-Fixed State to Global Trajectory

2.5의 모델은 제어기 설계에는 적합하지만, 그 상태만으로는 차가 전역 좌표계에서 어디를 지나갔는지 바로 보이지 않는다. 2.7은 이 간격을 메운다. 도로 중심선 위 기준점의 전역 좌표를 `X_{des}`, `Y_{des}`라고 두고, 차량의 횡오차 `e_1`를 이용하면 차량의 위치는 다음처럼 복원된다.

$$
X = X_{\mathrm{des}} - e_1 \sin \psi
$$

$$
Y = Y_{\mathrm{des}} + e_1 \cos \psi
$$

여기서 기준 경로는 desired heading을 적분해 얻는다.

$$
X_{\mathrm{des}} = \int_0^t V_x \cos \psi_{\mathrm{des}} \, dt,
\qquad
Y_{\mathrm{des}} = \int_0^t V_x \sin \psi_{\mathrm{des}} \, dt
$$

또 `\psi = \psi_{\mathrm{des}} + e_2`를 대입하면 road-relative simulation 결과를 그대로 global `X-Y` trajectory로 옮길 수 있다. 제어기 입장에서는 error state만 보면 되지만, 해석과 시각화 단계에서는 결국 전역 궤적 복원이 필요하다는 뜻이다.

## 2.8 Road Model and Clothoids

2.8은 차량 모델만큼이나 중요한 "도로 모델"을 다룬다. 기본 개념은 곡률 `\kappa = 1 / R`이다. 도로가 갑자기 직선에서 급한 원호로 바뀌면 steering demand와 lateral acceleration도 급격히 바뀌기 때문에, 실제 추종 가능한 reference를 만들려면 곡률 연속성이 중요하다.

이때 쓰는 대표 도구가 clothoid spiral이다. clothoid는 arc length가 늘어날수록 곡률이 선형적으로 변하는 곡선으로, 직선과 원호 사이를 부드럽게 연결할 때 자주 등장한다. 책은 이 곡선을 Fresnel integrals로 표현하고, 왜 highway geometry에서 transition curve로 유용한지를 설명한다. 핵심은 road model이 단순한 배경이 아니라 controller가 실제로 따라가야 할 입력 신호 생성기라는 점이다.

## 2.9 What the Chapter Leaves You With

이 장이 남기는 lateral model family는 네 가지다.

1. 저속 기하 관계를 설명하는 kinematic vehicle model
2. 횡위치와 요각을 상태로 두는 dynamic bicycle model
3. lane keeping에 직접 연결되는 road-error model
4. yaw stability 해석에 직접 연결되는 `\beta-r` model

여기에 2.7의 좌표 복원과 2.8의 road model이 붙으면서, 제어기 내부 상태와 바깥 세계의 주행 궤적이 하나로 이어진다. Chapter 2는 결국 "차량 횡운동을 어떤 상태로 볼 것인가"와 "그 상태를 어떤 도로 위에서 해석할 것인가"를 동시에 정리하는 장이다.

## What To Pay Attention To

- `\beta`는 vehicle slip angle이고, tire slip angle `\alpha_f`, `\alpha_r`와 다르다.
- `e_1`, `e_2`는 road-relative tracking error이고, `\beta`, `r`는 차량 자체의 횡안정성 state다.
- `V_x r` 항은 임의 보정이 아니라 rotating frame에서 inertial acceleration을 계산한 결과다.
- trajectory reconstruction이 있어야 error-state simulation을 실제 주행 궤적으로 읽을 수 있다.
- road curvature continuity가 있어야 lane-keeping reference가 물리적으로 자연스럽다.

## What To Build From Here

이 장의 최소 구현은 constant-curvature road에서 lane-relative error model과 `\beta-r` model을 같은 파라미터와 steering input으로 비교하는 시뮬레이터다. 여기에 2.7의 식을 붙이면 `e_1`, `e_2`, `\beta`, `r`뿐 아니라 `X`, `Y`, `\psi`까지 복원할 수 있고, 2.8의 clothoid road를 붙이면 reference path 자체를 더 현실적으로 만들 수 있다.

## Bridge to Chapter 3

Chapter 3는 lane keeping controller를 직접 다루지만, 그 출발점은 이미 여기서 정해졌다.

- lane keeping을 위한 상태 선택: `e_1`, `\dot{e}_1`, `e_2`, `\dot{e}_2`
- yaw stability를 위한 상태 선택: `\beta`, `r`
- road curvature와 bank angle을 입력/교란으로 읽는 관점
- body-fixed state를 global trajectory로 되돌리는 좌표 복원 단계
