<h1 align="center">🤖 로봇은 없지만, 로봇은 만들고 싶어</h1>

<div align="center">
<a href="https://pseudo-lab.com"><img src="https://img.shields.io/badge/PseudoLab-S13-3776AB" alt="PseudoLab"/></a>
<a href="https://discord.gg/EPurkHVtp2"><img src="https://img.shields.io/badge/Discord-BF40BF" alt="Discord Community"/></a>
<a href="https://github.com/dlwldn4824/robot-without-robot/stargazers"><img src="https://img.shields.io/github/stars/dlwldn4824/robot-without-robot" alt="Stars Badge"/></a>
<a href="https://github.com/dlwldn4824/robot-without-robot/network/members"><img src="https://img.shields.io/github/forks/dlwldn4824/robot-without-robot" alt="Forks Badge"/></a>
<a href="https://github.com/dlwldn4824/robot-without-robot/pulls"><img src="https://img.shields.io/github/issues-pr/dlwldn4824/robot-without-robot" alt="Pull Requests Badge"/></a>
<a href="https://github.com/dlwldn4824/robot-without-robot/issues"><img src="https://img.shields.io/github/issues/dlwldn4824/robot-without-robot" alt="Issues Badge"/></a>
<a href="https://github.com/dlwldn4824/robot-without-robot/graphs/contributors"><img alt="GitHub contributors" src="https://img.shields.io/github/contributors/dlwldn4824/robot-without-robot?color=2b9348"></a>
</div>
<br>

> **실물 로봇 없이 시작하는 AI × Robotics 입문 프로젝트**

로봇에 관심은 있지만 어디서부터 시작해야 할지 몰랐던 사람들이 모여, **Robotics를 0부터 함께 공부하고 직접 움직여보는 프로젝트**입니다.

실물 로봇 대신 ROS 2와 Simulation 환경을 활용하여 로봇의 기본 원리부터 센서, SLAM, Navigation, Computer Vision, AI × Robotics까지 단계적으로 경험합니다.

그리고 이 과정에서 ChatGPT와 Codex 같은 AI를 적극적으로 활용합니다.

**“로봇을 처음 접하는 사람들이 AI와 함께하면 어디까지 갈 수 있을까?”**

완성된 결과뿐 아니라 처음 접한 개념, 막혔던 부분, AI가 잘못 알려준 내용, 실패한 코드와 해결 과정까지 함께 기록합니다.

---

# 🕹️ 지금 바로 해보기 — 눈 감고 주차하기

**[▶ 브라우저에서 열기](https://dlwldn4824.github.io/robot-without-robot/booth/)** · 설치 없음 · 60초

방향키로 로봇을 몰아 주차 칸에 넣는 게임입니다. 단, 화면에 보이는 로봇은 실제 위치가 아니라
**로봇이 자기가 있다고 믿는 위치**입니다. 달릴수록 파란 원이 커지는데, 그게 로봇이 자기 위치를
모르게 되는 정도입니다. 다 왔다 싶을 때 스페이스바를 누르면 진짜 위치가 나옵니다.

SLAM을 켜면 벽의 표식이 보일 때마다 원이 다시 줄어듭니다. 그 차이가 이 게임의 전부입니다.

바퀴 명령과 센서 변환은 설명용으로 새로 짠 코드가 아니라, MCU에 올리는
[`w6-firmware-c/firmware.c`](w6-firmware-c/firmware.c)를 wasm으로 컴파일해 그대로 부르고 있습니다.
4.3 KB, 정수 연산만 쓰고, 외부 의존성이 없습니다.

> 2026 AWS Community Day Seoul 가짜연구소 부스에서 쓰려고 만들었습니다. 코드는 [`booth/`](booth/)에 있습니다.

---

# ✨ Why this project?

## 로봇 한번 해보고 싶은데, 뭐부터 해야 하지?

로봇에 관심이 생겨 찾아보기 시작하면 ROS 2, 좌표계, Kinematics, LiDAR, SLAM, Navigation 등 수많은 개념을 만나게 됩니다.

그리고 곧 또 다른 문제가 생깁니다.

**“그래서 로봇을 하나 사야 하나?”**

이 프로젝트에서는 일단 사지 않습니다.

실물 하드웨어 대신 Simulation 환경에서 시작하여 Robotics의 핵심 개념을 직접 실행해보고 이해하는 것을 목표로 합니다.

또한 누군가가 일방적으로 Robotics를 가르치는 수업이 아닙니다.

**빌더를 포함한 참여자들이 함께 0부터 공부하고, 매주 작은 Robot Challenge를 해결하면서 성장하는 프로젝트**를 지향합니다.

### 왜 실물 로봇 없이 하나요?

- 입문 단계에서 가장 비싼 장벽은 하드웨어가 아니라 **개념을 손으로 만져보는 경험**입니다.
- Simulation이면 좌표계, 센서, SLAM, Navigation을 같은 환경에서 반복 실험할 수 있습니다.
- 실패 비용이 낮아서, 망가진 코드와 이상한 궤적도 학습 기록이 됩니다.
- 나중에 실물 로봇을 만나더라도, 이미 머리와 손으로 한 번 지나온 상태가 됩니다.

### 왜 AI와 같이 배우나요?

로봇을 처음 공부하면 문서도 많고, 에러 메시지도 많고, “이게 맞나?” 싶은 순간도 많습니다.

그래서 이 프로젝트는 AI를 정답 생성기가 아니라 **같이 헤매는 동료**로 씁니다.

- 모르는 개념을 빠르게 물어보고
- 예제 코드를 함께 작성하고
- 틀린 답변은 의심하고
- 그 과정까지 기록합니다.

AI가 잘 도와준 순간보다, **AI가 틀린 이유를 우리가 찾아낸 순간**이 더 좋은 학습입니다.

---

# 🧪 How We Learn

> **Learn → Build → Break → Share**
>
> 이론을 공부하고 → 직접 구현하고 → 실패해보고 → 서로 시행착오를 공유합니다.

### ① Learn

이번 주 Robot Challenge에 필요한 Robotics 개념을 공부합니다.

### ② Build

ROS 2와 Simulation 환경에서 직접 구현합니다.

### ③ Break

실패하고, 오류를 만나고, AI의 답변도 의심해봅니다.

### ④ Share

어떻게 해결했는지, 무엇이 잘못됐는지 함께 공유합니다.

매주 완벽한 결과물을 만드는 것보다 **직접 시도한 기록 하나를 남기는 것**을 중요하게 생각합니다.

```text
개념 공부
    ↓
시뮬레이터에서 실행
    ↓
에러 / 이상한 움직임 / AI의 틀린 답
    ↓
원인 찾기
    ↓
주간 기록으로 공유
```

---

# 🎯 Goal

### 우리가 이번 시즌에 만들고 싶은 것

14주 동안 Robotics를 0부터 쌓아, 시뮬레이터 안에서 로봇이 스스로 움직이게 만드는 것이 목표입니다.

* [ ] **Milestone 1. Robotics 기초** — 좌표계, 변환, Kinematics 등 로봇이 움직이기 위한 기본 언어를 이해한다.
* [ ] **Milestone 2. ROS 2 / Simulation** — ROS 2 노드·토픽·센서를 다루고, 시뮬레이터에서 로봇을 띄워 조작한다.
* [ ] **Milestone 3. Autonomous Robot** — SLAM으로 지도를 만들고, Navigation으로 목표 지점까지 가게 한다.
* [ ] **Milestone 4. AI × Robotics** — 카메라와 인식 모델을 연결해, 본 것을 바탕으로 로봇이 행동하게 한다.
* [ ] **Milestone 5. Final Challenge** — 팀이 정한 미션을 시뮬레이터에서 시연하고, 과정과 실패까지 공개한다.

### 📦 Expected Outcome

최종적으로 다음과 같은 결과물을 만드는 것을 목표로 합니다.

* `Weekly Challenges` — 매주 작은 Robot Challenge와 학습/실패 기록
* `Robot Simulation Demo` — SLAM · Navigation · AI 인식이 연결된 시뮬레이션 데모
* `Beginner Guide` — 초보자가 같은 길을 따라올 수 있는 입문 가이드

> 완벽한 로봇을 만드는 것보다, **처음 배우는 사람이 따라올 수 있는 과정**을 남기는 것을 성공으로 봅니다.

---

# 🗺️ Weekly Roadmap

> **프로젝트의 14주 여정**
>
> 일정은 Season 13 기준으로 예정되어 있으며, 모임 요일·시간과 Magical Week는 확정 후 공지합니다.

| Week | 날짜 | Milestone | 주요 활동 | Challenge / 결과물 |
| --- | --- | --- | --- | --- |
| **W01** | 2026.10.07 | Robotics 기초 | OT, 목표 공유, 환경 세팅, 로봇이란 무엇인가 | 개발 환경 세팅 + 시뮬레이터 첫 실행 |
| **W02** | 2026.10.14 | Robotics 기초 | 좌표계, TF, 변환 | 두 좌표계 사이 변환을 시각화하기 |
| **W03** | 2026.10.21 | Robotics 기초 | Kinematics, 로봇이 움직이는 원리 | 차동 구동 로봇이 회전·직진하는 이유 설명 + 간단 실험 |
| **W04** | 2026.10.28 | ROS 2 / Simulation | ROS 2 입문 (node, topic, service, launch) | 토픽으로 `Hello Robot` 주고받기 |
| **W05** | 2026.11.04 | ROS 2 / Simulation | Simulation 환경 구축 | 시뮬레이터에 로봇을 스폰하고 텔레옵으로 움직이기 |
| **W06** | 2026.11.11 | ROS 2 / Simulation | 센서 다루기 (LiDAR, Camera, IMU) | 센서 데이터를 시각화하고 한 줄로 해석하기 |
| **W07** | 2026.11.18 | Autonomous Robot | 반응형 제어 | 벽 따라가기 또는 장애물 앞에서 멈추기 |
| **W08** | 2026.11.25 | Autonomous Robot | SLAM | 시뮬레이션 맵 한 장 만들기 |
| **W09** | 2026.12.02 | Autonomous Robot | Navigation | 목표 지점까지 자율 주행시키기 |
| **W10** | 2026.12.09 | AI × Robotics | Computer Vision for Robots | 카메라로 물체 인식하기 |
| **W11** | 2026.12.16 | AI × Robotics | Perception + Action | 인식한 대상을 향해 가거나 피하기 |
| **W12** | 2026.12.23 | Final Challenge | 미션 설계와 시스템 통합 | Final Challenge 스코프 확정 + 통합 초안 |
| **W13** | 2026.12.30 | Final Challenge | 구현, 디버깅, 실패 기록 | Demo 초안 + 막힌 지점 기록 |
| **W14** | 2027.01.06 | Final Challenge | 발표, 회고, Archive 정리 | 시뮬레이션 데모 + 초보자 가이드 + 회고 |

가짜연구소 Magical Week 기간에는 정기 모임이 조정될 수 있습니다.

---

# 👥 Team

## Core Team

입문자가 함께 배우는 프로젝트인 만큼, 역할보다 **매주 Challenge를 끝까지 시도하는 것**을 더 중요하게 생각합니다.

| Role | Name | GitHub | 담당 |
| --- | --- | --- | --- |
| 🧭 Builder | 이지우 | [@dlwldn4824](https://github.com/dlwldn4824) | 프로젝트 기획 · 운영 · 함께 0부터 배우기 |
| 🏃 Runner | `@name` | | `__________` |
| 🏃 Runner | `@name` | | `__________` |
| 🏃 Runner | `@name` | | `__________` |
| 🏃 Runner | `@name` | | `__________` |

---

# 📚 Archive

활동하면서 `docs/`, `challenges/`, `projects/`, `resources/` 아래에 주차별 자료, 코드, 발표자료를 쌓아갑니다.

지금은 계획 공개 단계이며, 결과물 링크는 이후 채워집니다.

## 결과물

* 🔗 Repository: [dlwldn4824/robot-without-robot](https://github.com/dlwldn4824/robot-without-robot)
* 🌐 Robot Simulation Demo: [눈 감고 주차하기](https://dlwldn4824.github.io/robot-without-robot/booth/)
* 📝 Beginner Guide / Blog: `TBA`
* 🎥 Presentation: `TBA`

## 주차별 기록

| Week | 자료 | 코드 | 발표 / 공유 |
| --- | --- | --- | --- |
| W01 | `TBA` | `TBA` | `TBA` |
| W02 | `TBA` | `TBA` | `TBA` |
| W03 | `TBA` | `TBA` | `TBA` |
| W04 | `TBA` | `TBA` | `TBA` |
| W05 | `TBA` | `TBA` | `TBA` |
| W06 | [Sensor + Control](w6/W6-sensor-control.md) | [`w6/`](w6/) · [`w6-embedded/`](w6-embedded/) · [`w6-firmware-c/`](w6-firmware-c/) | [스터디 로그](studylog/) |
| W07 | [SLAM](studylog-w7/) | [`w7/`](w7/) | [스터디 로그](studylog-w7/) |
| W08 | `TBA` | `TBA` | `TBA` |
| W09 | `TBA` | `TBA` | `TBA` |
| W10 | `TBA` | `TBA` | `TBA` |
| W11 | `TBA` | `TBA` | `TBA` |
| W12 | `TBA` | `TBA` | `TBA` |
| W13 | `TBA` | `TBA` | `TBA` |
| W14 | `TBA` | `TBA` | `TBA` |

## 폴더 구조

```text
/
├── booth/              AWS Community Day 부스용 게임 (단독 실행 index.html)
├── w6/                 W6 Sensor + Control — MuJoCo 장애물 회피
├── w6-embedded/        같은 제어기를 MCU 시뮬레이터 위에 올린 것
├── w6-firmware-c/      C 펌웨어 (dylib · wasm · 보드용, 한 파일에서 세 갈래)
├── w7/                 W7 SLAM — LiDAR 격자 지도, EKF-SLAM, 다중 지점 매핑
├── studylog/           W6 스터디 로그 (HTML)
└── studylog-w7/        W7 스터디 로그 (HTML)
```

PythonRobotics 예제를 불러다 쓰는 코드가 있습니다. 저장소에는 담지 않았으니 옆에 따로 받아 두세요.

```bash
git clone https://github.com/AtsushiSakai/PythonRobotics.git
```

## 돌려보기

```bash
# W6 — 명세대로 만든 컨트롤러는 40초에 61번 부딪히고, 고친 쪽은 0번입니다
cd w6 && pip install mujoco imageio
python run.py --duration 40
python run.py --controller improved --duration 40

# W7 — 같은 주행에서 바퀴만 쓰면 5.54 m, 관측을 더하면 0.70 m 틀어집니다
cd ../w7 && python ekf_run.py
python temi_map.py        # 돌아다니며 지도를 쌓을 때와 위치가 틀어졌을 때

# 부스 게임 — 서버도 인터넷도 필요 없습니다
open ../booth/index.html
```

---

# 🌱 How to Engage

이런 분들과 함께하고 싶습니다.

- 로봇은 궁금한데, 하드웨어를 살 자신은 아직 없는 분
- ROS 2나 Simulation을 처음 접해도 매주 손으로 따라가볼 분
- AI를 쓰되, 답을 그대로 믿지 않고 같이 검증해볼 분
- 완성본보다 **막혔던 기록**을 남기는 데 거리낌 없는 분

참여 방식은 다음과 같습니다.

- 🧭 **Builder로 참여** — 프로젝트 기획 · 운영, 그리고 러너와 함께 0부터 공부
- 🏃 **Runner로 참여** — 매주 Learn → Build → Break → Share, Challenge 수행
- 👀 **청강 참여** — 공개 세션에서 구경하고 질문하기

❗️참여 링크: [가짜연구소 디스코드](https://discord.gg/EPurkHVtp2)
❗️커뮤니케이션 채널: 디스코드 `#{{채널명}}` (확정 후 공지)

**누구나 청강을 통해 모임에 참여하실 수 있습니다.**

1. 특별한 신청 없이 정기 모임 시간에 맞추어 디스코드 채널로 입장
2. Magical Week 중 행사에 참가
3. Pseudo Lab 행사에서 만나기

---

# 🙏 Acknowledgement

이 프로젝트는 가짜연구소 Open Academy로 진행됩니다.
여러분의 참여와 기여가 ‘우연한 혁명(Serendipity Revolution)’을 가능하게 합니다. 모두에게 깊은 감사를 전합니다.

This project is developed as part of Pseudo-Lab's Open Research Initiative. Special thanks to our contributors and the open source community for their valuable insights and contributions.

---

# 👋 About Pseudo Lab

[Pseudo-Lab](https://pseudo-lab.com/) is a non-profit organization focused on advancing machine learning and AI technologies. Our core values of Sharing, Motivation, and Collaborative Joy drive us to create impactful open-source projects. With over 5k+ researchers, we are committed to advancing machine learning and AI technologies.

<h2>Contributors 😃</h2>
<a href="https://github.com/dlwldn4824/robot-without-robot/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=dlwldn4824/robot-without-robot" />
</a>
<br><br>

<h2>License 🗞</h2>

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).
