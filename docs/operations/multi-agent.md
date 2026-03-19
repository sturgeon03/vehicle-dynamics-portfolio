# Multi-Agent Ops

이 프로젝트는 한 에이전트에 번역, 수식, QA, 발행을 모두 몰지 않는다. 좋은 결과는 역할을 분리할 때 나온다.

## Core Roles

- Parser Agent: PDF 구조, chapter/section/figure 인덱스 추출
- Translation Agent: 섹션 단위 번역 초안 생성
- Terminology Agent: glossary와 용어 통일 관리
- Math Agent: 식, 기호, LaTeX 표현 검수
- Pedagogy Agent: 직관 설명, 비유, FAQ 작성
- Simulation Agent: 코드/그래프/재현 실험 초안 생성
- QA Agent: 누락, 식 번호, 변수명, 오역 후보 점검
- Publisher Agent: frontmatter, nav, internal linking, 공개 문안 정리

## Ownership Rules

- Translation Agent는 `study-assets/chapters/*/translation.md`만 우선 책임진다.
- Math Agent는 `equations.md`와 glossary 관련 표현만 본다.
- Publisher Agent는 `docs/chapters/`와 `docs/tracks/`만 다룬다.
- 동시에 여러 에이전트가 작업할 때는 같은 파일을 건드리지 않는다.

## Done Criteria

- glossary 충돌 없음
- 핵심 식/변수 설명 완료
- 공개용 페이지에 설명 또는 구현 산출물 포함
- review checklist의 blocking issue 해소

