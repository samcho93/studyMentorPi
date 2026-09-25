# studyMentorPi — MentorPi 이동로봇 인터랙티브 강의

Hiwonder **MentorPi**(메카넘/애커만, Raspberry Pi 5, ROS 2 Humble)로 배우는 이동로봇 한국어 강의 사이트.

- 사이트: https://samcho93.github.io/studyMentorPi/
- 4개 트랙 46챕터: 기초·하드웨어(B) · ROS 2·OpenCV(R) · 이동로봇 핵심(M) · 비전 AI·자율주행(A)
- 도구: URDF 기반 3D 시뮬레이터, 2D 시뮬레이터(주행·라이다·SLAM·내비·라인), ROS 2 Playground(Pyodide + rclpy shim), 섀시 기구학 실험실, HSV 색 실험실, URDF 뷰어

## 로컬 실행

```bash
python build.py --serve    # http://localhost:8000
```

구조와 작성 규칙은 [CLAUDE.md](CLAUDE.md), 원본 교재 매핑은 [docs/SOURCES.md](docs/SOURCES.md).
