# Chapter 3. Steering Control for Automated Lane Keeping

> 시작 페이지: p.74  
> 트랙: Track A - Lateral Dynamics and Lane Keeping

## Why This Chapter Matters

Chapter 2가 lane keeping용 lateral model을 만들었다면, Chapter 3의 출발점은 그 모델을 실제 steering law로 바꾸는 일이다. 여기서 중요한 전환은 단순히 "차량이 곡선을 돈다"를 설명하는 수준을 넘어, "어떤 조향 입력을 주면 차선 중심을 안정적으로 따라가게 되는가"를 폐루프 관점에서 묻는다는 점이다. 3.1과 3.2는 먼저 full-state feedback으로 폐루프를 안정화하고, 곡률이 있는 도로에서는 왜 안정화만으로 오차가 모두 사라지지 않는지 보여 준다. 3.3은 그 결론을 steady-state cornering의 기하학과 타이어 횡력 균형으로 다시 읽어, 왜 필요한 조향각이 속도와 차량 handling 특성에 따라 달라지는지 설명한다. 3.4는 차량 속도가 변해도 왜 하나의 constant gain이 일정 범위에서는 여전히 유효할 수 있는지를 보여 주고, 3.5-3.6은 그 기준선을 실제 preview measurement와 unity feedback loop로 옮겨 output-feedback 설계의 출발점을 만든다.

## The Chapter Opening

- 3.1은 Chapter 2의 road-relative state-space model `\dot{x} = A x + B_1 \delta + B_2 \dot{\psi}_{\mathrm{des}}`를 바로 제어기 설계 문제로 넘긴다.
- 상태는 `x = [e_1, \dot{e}_1, e_2, \dot{e}_2]^T`로 잡아 lateral position error와 heading-related error를 함께 본다.
- steering input `\delta`로 폐루프 pole을 옮길 수 있다는 사실이 lane keeping 제어기의 첫 번째 설계 자유도다.
- 하지만 `\dot{\psi}_{\mathrm{des}}`는 도로 곡률이 만드는 지속 입력이므로, 안정한 폐루프와 zero steady-state tracking은 같은 문제가 아니다.
- 3.3은 그 결과를 steady-state cornering 관점에서 다시 풀어, `\delta_{\mathrm{ss}} = L/R + K_V a_y`가 단순 공식이 아니라 타이어 슬립과 force balance의 요약이라는 점을 보여 준다.
- 3.4는 longitudinal speed가 변할 때도 같은 full-state feedback gain이 작동할 수 있는 조건을 Lyapunov 관점에서 정리해, 이후 output-feedback sections의 설계 감각을 준비시킨다.
- 3.5는 실제 센서가 c.g. 앞쪽 preview point에서 lateral position을 측정한다는 사실을 이용해, full-state controller가 실제로는 어떤 measured output을 보게 되는지 정의한다.
- 3.6은 그 measured output을 unity feedback loop의 plant `P(s)`로 묶고, sensor location이 open-loop zero damping을 어떻게 바꾸는지 보여 준다.

이 장의 나머지 부분은 proportional controller, lead compensator, sensor-location trade-off 같은 output-feedback 분석으로 이어진다. 공개 초안에서는 먼저 full-state 기준선과 steady-state cornering 직관을 세운 뒤, 3.5-3.6에서 그 기준선이 실제 preview measurement와 unity feedback loop로 어떻게 넘어가는지까지 정리하는 것이 핵심이다.

## 3.1 State Feedback as the Baseline Lane-Keeping Controller

3.1의 기본 제어 law는 가장 직접적이다.

$$
\delta = -Kx = -k_1 e_1 - k_2 \dot{e}_1 - k_3 e_2 - k_4 \dot{e}_2
$$

이 식의 의미는 단순하다. 조향 입력을 lateral displacement, lateral velocity, heading mismatch, yaw-related motion의 선형 결합으로 만들면, Chapter 2에서 얻은 plant를 표준 state-feedback 문제로 다룰 수 있다. 본문은 `(A, B_1)`가 controllable이라는 점을 바탕으로, 원하는 closed-loop pole 위치를 설계자가 정할 수 있음을 보여 준다. lane keeping이 갑자기 특별한 문제로 남는 것이 아니라, 잘 정의된 상태공간 모델 위에서 pole placement를 수행하는 문제로 바뀌는 것이다.

여기서 중요한 배경은 open-loop lateral plant 자체가 자연스럽게 안정하지 않다는 점이다. 책은 open-loop matrix가 원점에 있는 고유값을 포함해 그대로 두면 lane-relative states가 스스로 정렬되지 않는다고 지적한다. 즉, 도로 중심선에 맞춰 주행하고 싶다면 steering feedback이 단순 보정이 아니라 시스템을 제어 가능한 형태로 만드는 기본 조건이다.

시뮬레이션 해석도 이 메시지에 맞춰 읽을 수 있다. 차량이 직선도로에서 시작해 큰 반경의 곡선도로로 들어갈 때, 적절한 pole placement는 lateral error와 yaw-angle error의 transient를 빠르게 가라앉힌다. 그러나 응답이 "안정하다"는 사실이 곧 "모든 오차가 0으로 간다"는 뜻은 아니다. 곡선도로는 차량에 계속 원하는 yaw rate를 요구하고, 이 요구는 폐루프 안에서 외란처럼 사라지지 않는다.

## 3.2 Why Steady-State Error Survives on a Curve

3.2는 바로 이 지점을 파고든다. 제어기를

$$
\delta = -Kx + \delta_{\mathrm{ff}}
$$

처럼 state feedback과 curvature-aware feedforward의 합으로 두고, 일정 속도 `V_x`와 일정 곡률 반경 `R`을 갖는 도로에서 steady state를 계산한다. 이때 desired yaw rate는

$$
\dot{\psi}_{\mathrm{des}} = \frac{V_x}{R}
$$

로 고정된다.

핵심 결론은 비대칭적이다.

- 적절한 `\delta_{\mathrm{ff}}`를 고르면 steady-state lateral position error `e_{1,\mathrm{ss}}`는 0으로 만들 수 있다.
- 하지만 steady-state yaw-angle error `e_{2,\mathrm{ss}}`는 일반적으로 남는다.

이 결과는 lane keeping을 해석할 때 자주 놓치는 차이를 분명하게 만든다. 차량의 질량중심이 차선 중심을 잘 따라가는 것과, 차량 body heading이 도로 접선과 완전히 같아지는 것은 같은 요구가 아니다. 곡선 주행에서는 lateral tire force가 계속 필요하고, 그 힘을 만들기 위해 front/rear tire slip과 yaw attitude가 일정한 균형점을 형성한다. 따라서 "차선 중심에는 붙어 있지만 heading error는 약간 남는" steady state가 물리적으로 가능하다.

이 절의 실용적 산출물은 steady-state steering angle을 읽는 방식이다. zero lateral offset을 목표로 정리하면 필요한 steering angle은

$$
\delta_{\mathrm{ss}} = \frac{L}{R} + K_V a_y,
\qquad
a_y = \frac{V_x^2}{R}
$$

형태로 쓸 수 있다. 첫 항 `L / R`은 low-speed bicycle geometry에서 바로 나오는 순수 곡률 추종 조향각이고, 둘째 항 `K_V a_y`는 속도와 lateral acceleration이 올라갈수록 필요한 추가 조향량을 나타낸다. 여기서 `K_V`는 understeer gradient다. 즉, 3.2는 feedforward를 "보너스 항"이 아니라 "곡선도로가 이미 요구하는 기본 steering bias"로 읽게 만든다.

이 해석은 곧바로 3.3으로 이어진다. 3.2가 algebraic steady-state result를 주었다면, 다음 단계는 그 결과를 "왜 이런 조향각이 필요한가"라는 cornering physics로 다시 읽는 것이다.

## 3.3 Steady-State Cornering as Geometry Plus Handling

3.3의 역할은 3.2의 식을 다른 방식으로 다시 증명하는 데 그치지 않는다. 더 중요한 일은 `\delta_{\mathrm{ss}} = L/R + K_V a_y`를 물리적으로 해석 가능하게 만드는 것이다. 곡선도로를 일정 속도로 돌고 있을 때 차량은 단지 "원호를 따라간다"로 끝나지 않는다. front tire와 rear tire가 각자 필요한 slip angle을 만들고, 그에 대응하는 lateral tire force가 차량 질량중심의 원운동을 지탱해야 한다. 3.3은 바로 그 균형점의 의미를 풀어 준다.

### 3.3.1 Why the Steering Angle Has a Geometry Term and a Handling Term

steady-state cornering에서는 instantaneous turn center가 존재하고, front wheel과 rear wheel은 각자 조금 다른 방향의 속도벡터를 갖는다. 그래서 필요한 steering angle은 저속 자전거 모델처럼 단순히 `L/R`로 끝나지 않는다. 기하학만 보면 wheelbase `L`를 가진 차량이 반경 `R`의 원호를 따라가기 위해 필요한 기본 조향각은

$$
\delta_{\mathrm{geom}} = \frac{L}{R}
$$

처럼 읽힌다. 하지만 실제 고속 주행에서는 타이어가 횡력을 만들기 위해 slip angle을 가져야 한다. front tire와 rear tire의 slip contribution까지 포함하면 steady-state steering angle은

$$
\delta_{\mathrm{ss}} = \frac{L}{R} + K_V a_y,
\qquad
a_y = \frac{V_x^2}{R}
$$

가 된다. 이 식의 해석은 명확하다. 첫 항은 road curvature가 요구하는 순수 기하학 조향량이고, 둘째 항은 lateral acceleration을 만들기 위해 vehicle handling이 추가로 요구하는 조향 bias다.

여기서 `K_V`의 부호는 차량의 steady-state personality를 말해 준다.

- `K_V = 0`이면 neutral steer다. 속도가 바뀌어도 같은 반경의 곡선을 도는 데 필요한 steering angle은 거의 기하학 항 `L/R`로 정리된다.
- `K_V > 0`이면 understeer다. 속도가 올라가 lateral acceleration이 커질수록 더 큰 steering angle이 필요하다.
- `K_V < 0`이면 oversteer다. 속도가 올라갈수록 필요한 steering angle이 줄어들며, 극단적으로는 critical speed 개념까지 이어진다.

즉 3.3.1은 "왜 곡선 주행 조향각이 속도 의존적인가"를 설명하는 절이다. 속도에 따라 road radius 자체가 바뀌지 않아도, 타이어가 같은 곡률을 만들기 위해 요구하는 slip distribution이 달라지기 때문이다.

### 3.3.2 Why Zero Yaw-Angle Error Exists Only at One Speed

3.2에서 본 steady-state yaw-angle error `e_{2,\mathrm{ss}}`는 feedforward tuning으로 마음대로 없앨 수 있는 값이 아니었다. 3.3.2는 그 이유를 geometric interpretation으로 보여 준다. 차량 뒤쪽 절반이 원운동 중심에서 차지하는 기하학적 각도와, rear tire slip angle이 정확히 일치하는 순간에만 body heading과 road tangent가 steady state에서 같아질 수 있다.

중요한 점은 이 조건이 "특정한 한 속도"에서만 성립한다는 것이다. 다시 말해, zero yaw-angle error는 controller가 언제든 억지로 만들어 내는 성질이 아니라 차량 파라미터와 tire stiffness, 속도 조합이 우연히 맞아떨어질 때 생기는 특수한 operating point다. 그래서 같은 차량이라도 대부분의 속도에서는 `e_{2,\mathrm{ss}}`가 남는 편이 정상이다.

이 해석은 lane keeping 관점에서 매우 중요하다. path centerline을 잘 따라간다고 해서 body heading까지 항상 road tangent와 완전히 같아야 하는 것은 아니다. steady-state yaw error가 0이 되는 조건은 tracking law의 일반 목표라기보다, cornering geometry와 tire slip이 한 점에서 일치하는 예외적 상태에 가깝다.

### 3.3.3 Why Non-Zero Yaw-Angle Error Is Not Automatically a Failure

3.3.3의 메시지는 실용적이다. 곡선도로에서 `e_1`이 잘 억제되고 차량이 원하는 반경을 안정적으로 따라가는데도 `e_2`가 조금 남아 있다면, 그것만으로 제어가 실패했다고 결론내리면 안 된다. 그 잔차는 종종 타이어가 필요한 lateral force를 만들기 위해 요구되는 steady slip configuration의 표현이기 때문이다.

본문은 vehicle slip angle과 yaw-angle error를 연결해, body orientation과 velocity direction이 완전히 같지 않아도 차량이 정상적으로 곡선을 추종할 수 있음을 보여 준다. 직관적으로 말하면 차는 도로를 "빗겨 서서" 도는 것이 아니라, 필요한 tire force를 만들기 위해 아주 작은 자세 차이를 유지한 채 균형을 잡고 도는 것이다.

여기서 핵심은 어떤 오차가 performance error이고 어떤 오차가 equilibrium geometry의 일부인지를 구분하는 일이다.

- `e_1`가 커져 차선 중심을 놓친다면 tracking failure에 가깝다.
- 반면 `e_2`가 steady state에서 작고 예측 가능한 값으로 남는다면, 그것은 curved-road force balance의 결과일 수 있다.

그래서 Chapter 3의 steady-state 해석은 "오차를 모두 0으로 만드는 제어기"를 찾는 데서 끝나지 않는다. 어떤 오차는 반드시 없애야 하고, 어떤 오차는 차량 dynamics가 허용하는 정상 operating condition 안에서 읽어야 한다는 점을 구분하게 만든다.

## 3.4 Why One Constant Gain Can Still Work Over a Speed Range

3.4의 질문은 실용적이다. 지금까지의 설명은 특정한 `V_x`에서 plant를 고정한 뒤 state feedback을 설계하는 흐름이었다. 하지만 실제 차량은 속도가 계속 변한다. 그러면 lateral dynamics 행렬 `A(V_x)`와 input matrix `B_1(V_x)`도 speed-dependent가 되므로, "속도마다 gain을 다시 바꿔야 하나?"라는 질문이 자연스럽게 나온다.

이 절의 대답은 "항상 그런 것은 아니다"다. Chapter 3는 lateral plant가 speed range 안에서 convex하게 변한다는 점을 이용해, 하나의 constant feedback gain `K`로도 폐루프를 안정하게 유지할 수 있는 조건을 제시한다. 핵심은 모든 속도를 일일이 검사하는 대신, 속도 구간의 두 endpoint인 `V_{x,\min}`과 `V_{x,\max}`에서 같은 Lyapunov 함수가 감소하도록 만드는 것이다.

### What the Endpoint Lyapunov Test Is Really Checking

책의 정리는 closed-loop matrix

$$
A_{CL}(V_x) = A(V_x) - B_1(V_x)K
$$

를 놓고, 최소 속도와 최대 속도에서 얻는 두 행렬 `A_{\min}`, `A_{\max}`를 먼저 본다. 그리고 어떤 positive definite matrix `P`가 존재해서

$$
A_{\min}^T P + P A_{\min} < 0,
\qquad
A_{\max}^T P + P A_{\max} < 0
$$

가 둘 다 성립하면, 그 사이 속도 범위 전체에서도 안정성이 유지된다고 말한다.

이 조건의 물리적 의미는 단순하다. `V = x^T P x`라는 하나의 에너지-like measure를 잡았을 때, 느린 쪽 endpoint와 빠른 쪽 endpoint 모두에서 그 값이 감소하면, 그 사이에서 만들어지는 intermediate plant도 같은 measure를 계속 줄인다는 뜻이다. 3.4가 사용하는 convexity argument는 "중간 속도의 closed-loop matrix가 두 endpoint 행렬의 convex combination으로 쓸 수 있다"는 사실에 기대고 있다. 그래서 두 끝점에서 충분히 감쇠를 확보하면, 그 사이도 같은 안정성 certificate 안에 넣을 수 있다.

중요한 것은 이것이 단순한 수학 트릭이 아니라는 점이다. 설계자는 "모든 속도에서 pole이 왼쪽 반평면에 있다"는 식의 개별 확인 대신, 모든 속도에서 공통으로 통하는 Lyapunov 감소 법칙이 있는지를 보는 것이다. 즉 3.4의 endpoint test는 gain robustness를 확인하는 방식이다.

### Why This Matters for the Rest of Chapter 3

이 절은 Chapter 3의 앞부분과 뒷부분을 잇는 다리 역할을 한다. 3.1-3.3이 full-state feedback의 직관을 세워 주었다면, 3.4는 그 직관을 "속도가 변해도 유지되는가?"라는 더 실제적인 질문으로 확장한다. 이 확장은 이후 output-feedback sections에서 매우 중요해진다. 센서가 제한되고 observer나 dynamic compensator가 들어오더라도, 결국 설계자가 원하는 것은 operating range 전체에서 무너지지 않는 closed-loop structure이기 때문이다.

그래서 3.4는 gain scheduling을 본격적으로 시작하는 절이라기보다, "constant gain + common Lyapunov certificate"라는 가장 단단한 기준선을 제시하는 절로 읽는 편이 좋다. 먼저 full-state setting에서 속도 범위를 다루는 논리를 이해해야, 나중에 output-feedback controller를 설계할 때도 어떤 부분이 구조적 안정성이고 어떤 부분이 구현 제약인지를 분리해서 볼 수 있다.

## 3.5 Output Feedback Starts with a Look-Ahead Measurement

3.5는 제어기 구조를 바꾸기 전에 센서가 실제로 무엇을 측정하는지부터 다시 묻는다. 책의 설정에서 lateral position sensor는 차량 c.g. 위치를 직접 읽지 않고, 차량 앞쪽의 한 점에서 road centerline 대비 횡방향 위치를 측정한다. 이 preview point는 differential GPS, vision camera, magnetometer 같은 센서가 실제로 보는 지점에 더 가깝다.

small-angle 가정 아래에서는 이 측정 출력이 매우 간단한 식으로 정리된다.

$$
y = e_1 + d_s e_2
$$

여기서 `d_s`는 c.g. 앞쪽에 놓인 sensor location, 즉 look-ahead distance다. 이 식이 말하는 바는 분명하다. output `y`는 pure lateral position error가 아니라, lateral offset `e_1`과 heading error `e_2`가 preview geometry를 통해 합쳐진 measured output이다.

이 관점은 중요하다. 센서가 더 앞쪽에 있을수록 같은 heading mismatch도 더 큰 lateral deviation처럼 보인다. 따라서 3.5는 단순히 "state를 다 못 재니 output feedback을 쓴다"가 아니라, lane keeping sensor가 본질적으로 preview measurement를 제공한다는 점을 보여 준다. 이후 3.6의 unity feedback loop와 3.8의 lead compensator는 모두 이 출력식을 plant definition으로 삼아 전개된다.

## 3.6 Unity Feedback Loop Makes Sensor Location a Plant Property

3.6은 3.5에서 만든 measured output을 곧바로 block-diagram language로 넘긴다. 여기서 `P(s)`는 steering angle input에서 look-ahead measurement output까지의 plant transfer function이고, `C(s)`는 이후 설계할 controller transfer function이다. road curvature가 만드는 desired yaw rate는 `G(s)` 경로를 통해 들어오고, sensor noise `n(t)`도 같은 loop 안에서 명시된다.

이 절의 첫 번째 핵심 관찰은 `P(s)`의 pole-zero structure다. 책은 `d_s = 2 m`와 `d_s = 7 m`를 비교하며, plant가 원점의 두 pole, 한 쌍의 complex pole, 한 쌍의 complex zero를 가진다고 설명한다. 더 중요한 점은 sensor location이 커질수록 complex zero pair의 damping이 좋아진다는 사실이다.

이 의미는 단순하지 않다. `d_s`는 measurement equation에만 들어가는 geometry parameter가 아니라, controller가 마주하는 open-loop plant의 phase behavior까지 바꾼다. 따라서 3.6은 "센서를 어디에 두는가"가 이후 proportional controller나 lead compensator tuning의 난이도를 얼마나 바꾸는지 처음으로 드러내는 절이다.

또 하나 중요한 점은 3.6이 road input과 sensor noise를 같은 그림 안에 올려놓는다는 것이다. lane keeping은 reference curvature, measurement geometry, sensor noise, steering-to-output plant를 동시에 봐야 하는 문제이고, 3.6의 unity feedback loop는 그 네 요소를 한 화면에 올려놓는 공용 좌표계가 된다.

## What To Pay Attention To

- pole placement는 폐루프 안정화 도구이지, curved-road steady-state offset을 자동으로 제거하는 장치는 아니다.
- `e_1`와 `e_2`는 같은 tracking error family에 속하지만 steady state에서는 역할이 다르게 남는다.
- `L / R`은 기하학적 조향 요구량이고, `K_V a_y`는 고속 lateral dynamics가 추가로 요구하는 조향 보정량이다.
- `K_V`의 부호는 neutral steer, understeer, oversteer를 구분하며, speed sweep에서 steering demand가 어떻게 달라질지 결정한다.
- zero yaw-angle error는 일반 목표가 아니라 특정 속도에서만 가능한 특수한 cornering condition이다.
- non-zero `e_2`는 자동으로 controller failure를 뜻하지 않는다. path-tracking 성능과 steady-state tire-force equilibrium을 분리해서 읽어야 한다.
- `A(V_x)`와 `B_1(V_x)`가 speed-dependent여도, 같은 `K`와 같은 Lyapunov matrix `P`가 두 endpoint를 동시에 안정화하면 속도 구간 전체를 한 번에 다룰 수 있다.
- 3.4의 endpoint Lyapunov test는 "중간 속도를 다 계산해 본다"가 아니라 "공통 안정성 certificate가 두 끝점에서 성립하는가"를 확인하는 절차다.
- `y = e_1 + d_s e_2`는 c.g.에서의 pure lateral error가 아니라 look-ahead measurement다. `d_s`가 커질수록 heading error contribution도 같이 커진다.
- `P(s)`는 steering input에서 look-ahead measurement output까지의 plant다. sensor location `d_s`를 바꾸면 zero damping도 같이 바뀐다.
- 3.1-3.6은 후속 output-feedback sections의 benchmark 역할을 한다. 먼저 full-state 기준선과 steady-state cornering 직관, varying-speed stability 논리, preview measurement 구조, unity feedback loop를 이해해야 측정 가능한 출력만으로 무엇을 근사하는지도 분명해진다.

## What To Build From Here

이 구간의 최소 구현은 constant-curvature road 시뮬레이터에 varying-speed stability check를 얹는 형태다.

1. state feedback only
2. state feedback + curvature feedforward

같은 차량 파라미터와 속도에서 `e_1`, `e_2`, steering angle을 함께 그리면 3.1과 3.2의 메시지가 분명해진다. feedback only는 시스템을 안정하게 만들지만 steady lateral offset이 남을 수 있고, feedforward를 더하면 centerline tracking은 훨씬 좋아진다.

여기에 speed sweep 하나를 더 얹으면 3.3의 메시지까지 같이 볼 수 있다.

1. `\delta_{\mathrm{ss}}` 대 `V_x` 곡선
2. `K_V > 0`, `K_V = 0`, `K_V < 0` 세 경우 비교
3. `e_{2,\mathrm{ss}} = 0`이 되는 speed crossing 표시

그 위에 한 단계 더 얹을 수 있다.

1. `V_{x,\min}`과 `V_{x,\max}`에서 같은 `K`로 closed-loop matrix 구성
2. 공통 `P`를 찾거나 확인해 endpoint Lyapunov inequalities 검사
3. speed sweep 동안 같은 gain이 유지되는지 응답과 함께 비교

그러면 same-radius cornering에서도 steering demand가 왜 handling type에 따라 다르게 움직이는지, yaw-angle error가 왜 "있을 수도 있고 없어질 수도 있지만 일반적으로는 남는 값"인지, 그리고 왜 constant gain이 일정 속도 범위에서는 여전히 실용적인지까지 한눈에 읽을 수 있다.

여기에 3.5를 붙이면 simulation harness도 자연스럽게 확장된다.

1. `y = e_1 + d_s e_2` 출력 추가
2. `d_s = 2 m`, `d_s = 7 m` 같은 sensor location 비교
3. 같은 state trajectory라도 measured output이 어떻게 달라지는지 확인

그러면 다음 section의 unity feedback plant `P(s)`가 왜 sensor location에 민감한지도 미리 볼 수 있다.

여기서 한 단계 더 가면 3.6의 메시지를 직접 재현할 수 있다.

1. `d_s = 2 m`, `d_s = 7 m`에서 open-loop pole-zero map 계산
2. 같은 longitudinal speed에서 Bode magnitude/phase 비교
3. zero damping이 sensor location에 따라 어떻게 바뀌는지 기록

## Bridge to the Rest of Chapter 3

공개 초안 범위는 이제 3.6까지다. 여기서 이미 Chapter 3 전체의 문제 구조가 정해진다. lane keeping은 단순한 조향 gain tuning이 아니라, road-relative plant를 안정화하는 feedback과 curved-road steering demand를 미리 반영하는 feedforward를 어떻게 결합할지의 문제다. steady-state cornering 해석은 어떤 오차를 없애야 하고 어떤 오차는 dynamics의 일부로 해석해야 하는지 알려 주고, 3.4는 그 full-state 설계가 varying-speed operating range에서도 어떻게 유지될 수 있는지를 보여 준다. 3.5는 실제 센서가 보는 preview output `y = e_1 + d_s e_2`를 정의하고, 3.6은 그 output을 unity feedback plant `P(s)`로 묶어 sensor location이 loop dynamics를 어떻게 바꾸는지 보여 준다. 다음 우선순위는 3.7-3.8의 proportional / lead compensator analysis다.
