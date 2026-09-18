# README 그림 파일

| 파일 | 내용 | 사용처 |
|---|---|---|
| `architecture.svg` | 전체 시스템 아키텍처 — Leader(STM32F405 자체 FC) · ESP-NOW · Follower(Jetson + Pixhawk 6C) · TandemGCS | README `전체 시스템` |
| `tracking-failsafe.svg` | Depth 기반 추종 성공 시나리오와 검출 실패 시 Fail-safe 동작 | README `추종 동작과 Fail-safe` |

## 수정 방법

둘 다 손으로 작성한 SVG입니다. 빌드 과정도, 외부 폰트·스크립트 의존도 없습니다.
텍스트를 고치려면 파일을 열어 해당 `<text>` 값을 바꾸면 됩니다.

색은 파일 상단 `<style>`에 클래스로 모여 있고, `@media (prefers-color-scheme: dark)`
블록에 다크 모드 값이 따로 있어서 GitHub 라이트·다크 양쪽에서 읽힙니다. 색을 바꿀 때는
두 곳을 같이 손대야 합니다.

수정 후 확인:

```bash
python3 -c "import xml.dom.minidom as m; m.parse('docs/images/architecture.svg')"   # XML 유효성
```

그림이 실제로 어떻게 보이는지는 브라우저로 SVG 파일을 직접 열어보면 됩니다.
GitHub 본문 폭(약 860px)으로 줄어들어도 읽히도록 글자 크기를 잡아뒀으니,
텍스트를 크게 늘릴 때는 그 폭에서 확인하는 편이 좋습니다.

## 안정성 분석 그림 (`stability_*.png`)

`python3 analysis/stability_margins.py --plots` 가 생성합니다(손으로 고치지 않음). 보드 선도(`stability_bode_forward`),
스트링 안정성(`stability_string`), 4단 체인 응답(`stability_chain_step`), FC 시정수·지연 강건성(`stability_robustness`),
IMM-EKF 실측 주파수응답(`stability_ekf_frf`). 설명은 [../STABILITY_MARGINS.md](../STABILITY_MARGINS.md).
