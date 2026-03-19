# Chapter 2. Lateral Vehicle Dynamics

> 시작 페이지: p.42  
> 트랙: Track A - Lateral Dynamics and Lane Keeping

## Why This Chapter Matters

bicycle model, yaw rate, slip angle의 핵심을 시각적으로 설명한다.

## Section Map

- 2.1 Lateral systems under commercial development
- 2.2 Kinematic model of lateral vehicle motion
- 2.3 Bicycle model of lateral vehicle dynamics
- 2.4 Motion of a particle relative to a rotating frame
- 2.5 Dynamic model in terms of error with respect to road
- 2.6 Dynamic model in terms of yaw rate and slip angle
- 2.7 From body fixed to global coordinates
- 2.8 Road model
- 2.9 Chapter summary

## Public Deliverables

- 대표 개념 요약
- 핵심 식 시각화
- 구현 또는 시뮬레이션 링크
- 관련 chapter/track 연결

## Signature Output

bicycle model 해설과 상태 변수 시뮬레이터

## Pilot Focus

- bicycle model의 상태변수 정의를 한 번에 이해할 수 있게 정리
- yaw rate, side slip angle, lateral error의 관계를 도식화
- 차체 고정 좌표계와 global 좌표계 변환을 그림으로 설명
- lane keeping으로 넘어가기 위한 선수 개념을 명시

## Study Checklist

- [ ] 섹션 단위 번역 완료
- [ ] 핵심 식과 기호 정리
- [ ] 5줄 요약 작성
- [ ] 구현 노트 작성
- [ ] 공개용 해설 페이지 초안 작성

## Notes for Publication

- 이 페이지는 `study-assets/chapters/02-lateral-vehicle-dynamics/`에서 만든 자산을 바탕으로 다듬는다.
- 초안 단계에서는 원문 복제보다 설명, 도식, 코드, 비교를 우선한다.

## Planned Simulation

- 입력: 조향각, 종방향 속도, vehicle parameters
- 출력: yaw rate, slip angle, lateral displacement
- 최소 구현: 선형 bicycle model의 step steering response
