# CLAUDE.md — studyMentorPi : MentorPi 이동로봇 인터랙티브 강의 사이트

> 이 저장소에서 작업할 때 따르는 **프로젝트 헌장**. 구조·규칙·좌표 규약을 지킨다.
> 모호한 점은 추측으로 구조를 바꾸지 말고 `docs/DECISIONS.md`에 남긴다.

## 0. 한 줄 요약

GitHub Pages로 배포되는 **정적 강의 사이트**에서
① Hiwonder MentorPi(메카넘/애커만, Raspberry Pi 5, ROS 2 Humble) 공식 교재 18개 강좌를 4개 트랙 46챕터로 재구성한 한국어 강의,
② 섀시·라이다·SLAM·내비게이션·라인트레이싱을 돌려 보는 **2D 시뮬레이터**(`sim/`),
③ rclpy 호환 shim으로 ROS 2 노드 코드를 브라우저에서 실행하는 **ROS 2 Playground**(Pyodide, `python/mentorpi_sim`),
④ 섀시 기구학 실험실, HSV 색 실험실, URDF 뷰어를 제공한다.

- 참고 구현: `samcho93/studyDeltaRobot`, `samcho93/studyGo2` (content/*.md → build.py → lessons/*.html, sim/, tools/ 구조 계승)
- 배포 URL: `https://samcho93.github.io/studyMentorPi/`
- 원본 교재: `D:\MentorPi` (영문 PDF + 소스). 챕터 ↔ 원본 매핑은 `docs/SOURCES.md`.
- 언어: 본문·UI는 한국어, 코드·커밋은 영어(주석 한국어 허용).

## 1. 핵심 설계 원칙

1. **Static-first**: 서버 없이 GitHub Pages만으로 동작. npm 빌드 체인 금지. 라이브러리는 jsdelivr 버전 고정 URL.
2. **단일 진실 공급원**: 커리큘럼 `content/curriculum.json`. 섀시 파라미터는 원본 `controller/mecanum.py`, `ackermann.py` 값을 따른다.
3. **같은 코드, 두 타깃**: Playground의 `rclpy` shim은 실제 rclpy와 같은 import 경로·클래스·메서드 시그니처를 쓴다.
   학생 코드는 수정 없이 MentorPi 실물(ROS 2 Humble 컨테이너)에서 돌아가야 한다.
4. **안전**: 실물 주행 코드는 속도 제한(선속도 ≤ 0.2 m/s 권장, 각속도 ≤ 0.5 rad/s — 원본 app_cmd_vel_callback 기준)과 종료 시 정지 명령을 포함한다.

## 2. 좌표·파라미터 규약

- ROS REP-103: x 전방, y 좌측, z 위, yaw 반시계 +. 단위 SI.
- 메카넘: wheelbase 0.1368 m, track_width 0.1446 m, 바퀴 지름 0.065 m.
  `motor1 = vx - vy - ωz(a+b)/2` … (원본 mecanum.py, a=wheelbase, b=track_width, 결과에 [-m1,-m2,m3,m4] 부호).
- 애커만: wheelbase 0.145 m, track_width 0.133 m, 바퀴 지름 0.067 m, 최대 조향 29°,
  δ = atan(L·ω/v), 서보 펄스 = 1500 + 2000·deg(−δ)/180.
- TF: map → odom → base_footprint → base_link → laser_frame / imu_link / camera_link.

## 3. 디렉터리 구조

```
studyMentorPi/
├── build.py                 # content/*.md → lessons/*.html, index.html
├── content/curriculum.json  # 트랙: base(B) · ros(R) · mobile(M) · ai(A)
├── content/{base,ros,mobile,ai}/*.md, content/figures/*.svg
├── lessons/ index.html      # 빌드 산출물 — 직접 수정 금지
├── sim/                     # 2D 시뮬레이터 (주행·라이다·SLAM·내비·라인)
├── tools/                   # playground(+worker), kinematics-lab, color-lab, urdf-viewer
├── python/mentorpi_sim/     # Pyodide용 rclpy/msgs shim + 2D 월드
├── assets/                  # css / js / urdf
└── docs/                    # SOURCES, DECISIONS
```

## 4. 챕터 Markdown 규칙

front matter(`id, track, title, duration, level, requires, tools`) 후 섹션:
`## 학습 목표` → 이론/실습 → `## 자주 나는 오류와 해결`(필수) → `## 과제` → `## 참고자료`(필수).
커스텀 블록 `:::tip|info|warning|danger|safety|check|task|mission`, 그림 `@fig[name] 캡션`, 버튼 `@btn[~/sim/index.html] 라벨`,
실행 코드 ` ```python run `(Playground 열기 버튼), ` ```python robot `(실물 전용 배지), ` ```bash `, 수식 `$...$`.
실물 실습 섹션(`## 실습 (실물)`)에는 `:::safety` 필수.

## 5. 개발 명령

```bash
python build.py            # 빌드   (--serve 미리보기 :8000, --check 검사)
python tests/check_links.py
python python/make_manifest.py   # Playground에 마운트할 파일 목록 갱신 (python/ 수정 후)
```

## 6. 하지 말 것

- `lessons/`, `index.html` 직접 수정 · 서버 필요 기능 · `@latest` CDN
- 원본 교재 문장/그림을 그대로 복제 (요약·재구성하고 출처는 참고자료에 표기)
- 실물 속도 제한을 기본값으로 완화
