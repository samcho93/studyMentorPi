# 챕터 원고 작성 가이드 (studyMentorPi)

## 입력
- 커리큘럼(제목·요약·도구): `content/curriculum.json`
- 원본 매핑: `docs/SOURCES.md`
- 원본 텍스트(PDF 추출본): `C:\Users\samdo\AppData\Local\Temp\claude\D--MentorPi\bf87f053-3652-48d7-b329-7588f3017f31\scratchpad\txt\`
  파일명은 원본 경로의 `\`를 `__`로 바꾼 것 (예: `10. Lidar Lesson__Lesson 3 Lidar Obstacle Avoidance.txt`), 목록은 `_index.tsv`.
- 원본 소스 코드: `D:\MentorPi\Appendix\Source Code\src` (ROS 2 패키지: app, bringup, driver/controller, example, navigation, slam, peripherals, multi, yolov5_ros2 …),
  `D:\MentorPi\Appendix\Source Code\RRCLite_demo`, 각 강좌 폴더의 `*.py` 예제.
- 문체·형식 참고 원고: `C:\Users\samdo\AppData\Local\Temp\claude\D--MentorPi\bf87f053-3652-48d7-b329-7588f3017f31\scratchpad\studyDeltaRobot\content\control\r02.md`, `...\theory\t04.md`, 그림 예: `...\content\figures\r02-cycle.svg`
- Playground 실행 코드 규칙: `docs/PLAYGROUND_API.md` (**` ```python run ` 블록은 이 API만 사용**)

## 출력
- `content/<track>/<id>.md` (track = base | ros | mobile | ai), 그림은 `content/figures/<id>-<이름>.svg`
- 다른 파일(build.py, CSS, curriculum.json, 다른 챕터)은 수정하지 않는다. git 명령을 쓰지 않는다.

## 원고 형식
```
---
id: m06
track: mobile
title: (curriculum.json과 동일)
duration: 100
level: 중급
requires: [m05]
tools: [sim, playground]
---

## 학습 목표
- …(3~5개, 행동 동사로)

## (이론 섹션들 — ##/### 사용)
## 실습 (시뮬레이션)       ← 도구가 있는 챕터
## 실습 (실물)             ← 실물 로봇 절차. 반드시 :::safety 블록 포함
## 자주 나는 오류와 해결   ← 필수 (표 또는 목록, 증상 → 원인 → 해결)
## 과제                    ← 2~4개, 기본/심화
## 참고자료                ← 필수: 원본 교재 레슨명(영문 그대로) + 공식 문서 링크
```

## 내용 기준
- **번역이 아니라 강의**: 원본의 절차·명령·파라미터는 정확히 살리되, 왜 그렇게 하는지 원리·수식·그림으로 설명을 보강한다.
  원본 문장을 그대로 옮기지 않는다(요약·재구성). 원본에 없는 핵심 개념(예: 메카넘 역기구학 행렬 유도, AMCL 파티클 필터 단계, DWA 평가식)은 직접 추가한다.
- 분량: 챕터당 대략 250~450줄 markdown. 표·목록·콜아웃·코드를 적절히 섞는다.
- 명령어는 ` ```bash ` 블록. MentorPi 공식 이미지는 Docker 컨테이너 안에서 ROS 2 Humble을 쓴다(원본의 실제 명령·경로·launch 이름을 그대로 사용 — 예: `ros2 launch app lidar_node.launch.py` 같은 것은 원본/소스에서 확인해서 쓴다. 추측하지 말 것).
- 실물에서만 도는 코드는 ` ```python robot `, Playground에서 도는 코드는 ` ```python run `, 단순 조각은 ` ```python `.
- ` ```python run ` 블록 뒤에는 예상 출력 ` ```text ` 블록(대략값)을 붙인다. run 블록은 짧고(≤ 60줄) 30초 시뮬레이션 이내로.
- 커스텀 블록: `:::tip`, `:::info`, `:::warning`, `:::danger`, `:::safety`, `:::check`, `:::task`, `:::mission` (여는 줄에 제목을 쓸 수 있음, `:::`로 닫음).
- 버튼: `@btn[~/sim/index.html] 2D 시뮬레이터 열기`, `@btn[~/tools/kinematics-lab.html] …`, `@btn[~/tools/color-lab.html] …`, `@btn[~/tools/playground.html] …`, `@btn[~/tools/urdf-viewer.html] …`
- 수식: 인라인 `$v_x = …$` (굵은 수식은 표나 코드 블록으로).
- 그림: `@fig[m06-sectors] 캡션` → `content/figures/m06-sectors.svg`. 챕터당 1~3개. 손으로 쓴 간결한 SVG:
  `<svg viewBox="0 0 720 300" xmlns="http://www.w3.org/2000/svg" role="img" aria-label="…">`, 색은 직접 지정하지 말고 CSS 클래스 사용:
  `.lbl`(본문 13px) `.sub`(보조) `.tag`(작은 라벨) `.mono` `.hi`(강조 채움) `.wr` `.dg` `.ln`(선) `.ln-hi`(강조 선) `.box` `.box-hi`,
  또는 `style="fill:var(--accent)"`, `var(--text)`, `var(--text-faint)`, `var(--border)`, `var(--ok)`, `var(--warn)`, `var(--danger)`. 화살표 marker id는 파일마다 고유하게(`arr-<파일명>`).
  텍스트가 겹치거나 viewBox 밖으로 나가지 않게 좌표를 계산한다.
- 시뮬레이터(sim/index.html) 모드: 수동 주행(키보드/가상 조이스틱), 섀시 선택(메카넘/애커만), 라이다 표시, 라이다 회피·추종·경비, SLAM(점유격자 지도 작성), 내비게이션(목표 클릭 → A* + DWA/Pure Pursuit), 라인 트레이싱(트랙 월드), 오도메트리 오차 표시. 실습 절차에서 이 기능들을 활용하라.
- 한국어 존댓말 서술체(“~합니다”). 용어는 처음 나올 때 영문 병기.

## 검증
작성 후 `python build.py` 실행 → 자기 챕터에 "경고:" 줄이 없는지 확인(다른 챕터 "원고 없음"은 무시). 생성된 `lessons/<id>.html`을 열어 그림이 들어갔는지 grep으로 확인.
