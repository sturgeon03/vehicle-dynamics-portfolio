# Chapter 6. Adaptive Cruise Control

> 시작 페이지: p.168  
> 트랙: Track B - Longitudinal Dynamics and Automation

## Why This Chapter Matters

spacing policy와 string stability를 핵심 메시지로 묶는다.

## Section Map

- 6.1 Introduction
- 6.2 Vehicle following specifications
- 6.3 Control architecture
- 6.4 String stability
- 6.5 Autonomous control with constant spacing
- 6.6 Autonomous control with the constant time-gap policy
- 6.7 Transitional trajectories
- 6.8 Lower level controller
- 6.9 Chapter summary
- Appendix 6.A

## Public Deliverables

- 대표 개념 요약
- 핵심 식 시각화
- 구현 또는 시뮬레이션 링크
- 관련 chapter/track 연결

## Signature Output

ACC spacing policy 시뮬레이션

## Pilot Focus

- vehicle following specification을 제어 관점으로 재정리
- constant spacing과 constant time-gap policy를 비교
- string stability를 직관과 식 두 층으로 설명
- lower-level actuation과 upper-level spacing policy를 분리해서 설명

## Study Checklist

- [ ] 섹션 단위 번역 완료
- [ ] 핵심 식과 기호 정리
- [ ] 5줄 요약 작성
- [ ] 구현 노트 작성
- [ ] 공개용 해설 페이지 초안 작성

## Notes for Publication

- 이 페이지는 `study-assets/chapters/06-adaptive-cruise-control/`에서 만든 자산을 바탕으로 다듬는다.
- 초안 단계에서는 원문 복제보다 설명, 도식, 코드, 비교를 우선한다.

## Planned Simulation

- 입력: desired time gap, lead vehicle speed profile, ego vehicle delay
- 출력: spacing error, relative velocity, control input
- 최소 구현: CTG policy와 constant spacing policy 응답 비교
