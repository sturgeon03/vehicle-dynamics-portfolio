# Glossary

용어집은 책 전 범위에서 가장 중요한 일관성 도구다. 공개 페이지에는 정제된 정의를, 학습 자산에는 작업 중인 표현 후보를 남긴다.

## Starter Terms

| Term | Working Korean Term | Notes |
| --- | --- | --- |
| yaw rate | 요 레이트 | lateral dynamics 전반 |
| slip angle | 슬립각 | tire / lane keeping |
| vehicle slip angle | 차량 슬립각 | c.g. 속도 벡터와 차체 종축 사이 각 |
| tire slip angle | 타이어 슬립각 | `\alpha_f`, `\alpha_r` |
| bank angle | 노면 뱅크각 | lateral model disturbance |
| inertial coordinates | 관성 좌표계 | `XYZ`, Newtonian acceleration 기준 |
| body-fixed coordinates | 차체 고정 좌표계 | `xyz`, 차량과 함께 회전하는 좌표계 |
| global coordinates | 전역 좌표계 | `X`, `Y`로 표현하는 관성 공간상의 궤적 좌표 |
| road-relative error | 도로 기준 오차 | lane keeping error-state model |
| road centerline | 도로 중심선 | `e_1`을 정의하는 기준 경로 |
| lateral position error | 횡방향 위치 오차 | `e_1`, road centerline 기준 |
| orientation error | 자세 오차 | `e_2`, road tangent 기준 |
| curvature | 곡률 | `\kappa = 1/R`, road geometry 입력 |
| clothoid spiral | 클로소이드 곡선 | arc length에 따라 곡률이 선형으로 변하는 전이 곡선 |
| Fresnel integrals | 프레넬 적분 | clothoid를 매개화할 때 쓰는 적분 함수 |
| cornering stiffness | 코너링 강성 | tire force modeling |
| output feedback | 출력 피드백 | measured output `y` 기반 제어 |
| look-ahead distance | 선행 거리 | `d_s`, c.g. 앞쪽 sensor location |
| look-ahead lateral position measurement | 선행 지점 횡위치 측정 | `y = e_1 + d_s e_2` |
| unity feedback loop | 단위 피드백 루프 | `P(s)`와 `C(s)`의 표준 폐루프 구조 |
| plant transfer function | 플랜트 전달함수 | steering input에서 measured output까지 |
| sensor noise | 센서 노이즈 | 측정 output에 더해지는 `n(t)` |
| string stability | 스트링 안정성 | ACC / platooning |
| rollover | 전복 | roll dynamics |
| adaptive cruise control | 적응형 크루즈 컨트롤 | longitudinal control |
| automated highway systems | 자동화 고속도로 시스템 | traffic congestion research |
| intelligent transportation systems | 지능형 교통 시스템 | control + sensing + communication |
| hybrid electric vehicle | 하이브리드 전기차 | HEV |
| fuel cell vehicle | 연료전지 차량 | FCV |
