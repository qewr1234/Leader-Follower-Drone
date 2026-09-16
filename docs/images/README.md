# README 그림 파일

## 저장소에 있는 그림 (SVG, 바로 렌더링됨)

| 파일 | 내용 | 사용처 |
|---|---|---|
| `architecture.svg` | 전체 시스템 아키텍처 — Leader(STM32F405 자체 FC) · ESP-NOW · Follower(Jetson + Pixhawk 6C) · TandemGCS | README `전체 시스템` |
| `tracking-failsafe.svg` | Depth 기반 추종 성공 시나리오와 검출 실패 시 Fail-safe 동작 | README `추종 동작과 Fail-safe` |

두 파일 모두 손으로 작성한 SVG입니다. 텍스트를 고치려면 파일을 직접 열어 `<text>` 값을
수정하면 되고, 별도 빌드 과정은 없습니다. `@media (prefers-color-scheme: dark)`를 넣어
GitHub 다크 모드에서도 읽히도록 해뒀습니다.

수정 후 눈으로 확인하려면 브라우저로 파일을 열거나:

```bash
python3 -c "import xml.dom.minidom as m; m.parse('docs/images/architecture.svg')"   # XML 유효성
```

## 아직 넣지 않은 사진 (아래 이름으로 추가하면 활성화)

README의 갤러리 블록은 현재 HTML 주석으로 막아둔 상태입니다. 아래 파일을 이 디렉터리에
넣고 README에서 `실물 사진 · 화면 캡처 갤러리` 블록을 감싼 주석 기호(`<!--`, `-->`)만
지우면 그대로 렌더링됩니다.

| 파일 이름 | 내용 | 권장 규격 |
|---|---|---|
| `detection-ground-test.png` | Jetson 실기 지상 테스트 검출 화면 (bbox + conf + depth + IMM 오버레이) | 가로 800px 이상, PNG |
| `tandemgcs.png` | TandemGCS 운용 화면 (지도 · 센서 그래프 · 통신 상태) | 가로 1000px 이상, PNG |
| `follower-drone.jpg` | 팔로워 드론 실물 사진 | 가로 800px 이상, JPG |

한 장당 1MB 이하로 줄여서 넣는 것을 권합니다 (`git`에 바이너리로 들어가므로).

```bash
# 예: 화면 캡처 용량 줄이기
convert tandemgcs.png -resize 1400x -quality 85 docs/images/tandemgcs.png
```
