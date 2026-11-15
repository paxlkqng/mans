# MANS 네이티브 전환 감사

작성일: 2026-11-15

## 현재 구현

| 영역 | 확인된 구현 | 네이티브 전환 판단 |
| --- | --- | --- |
| 웹 인터페이스 | Vite + React + TypeScript. 계획과 항법 화면이 분리되어 있으며 Three.js 지형 뷰를 사용한다. | 보존한다. 데스크톱 앱은 별도 Rust 바이너리로 추가한다. |
| 경로 엔진 | `crates/mans-nav`의 Rust `astar`, `slope_degrees`, `predict`, `observe`. | 데스크톱 앱이 직접 링크해 재사용한다. |
| 웹 WASM | `wasm-bindgen` 래퍼 `plan_route`와 `public/wasm` 산출물. | 웹 호환용으로 유지한다. 데스크톱 앱에서는 사용하지 않는다. |
| 지형 데이터 | `data/sample-terrain.json`은 65 × 65 절차적 Jezero 유사 격자다. | 초기 데스크톱 렌더러의 검증 데이터로 사용한다. |
| NASA 데이터 처리 | `data/prepare_terrain.py`가 Rasterio GeoTIFF에서 65 × 65 로컬 격자 JSON을 만든다. | 입력 DTM 확보 후 같은 JSON 계약을 데스크톱 로더로 확장한다. |

## 웹 앱에서 확인한 동작

- `src/App.tsx`는 임무 계획 화면과 현장 항법 화면을 전환한다. 계획 화면은 지형 선택, 출발·도착점, 경로 정책, 고도 프로파일과 시나리오 제어를 제공한다.
- `src/TerrainScene.tsx`는 React Three Fiber로 변위된 지형, 경로, 선택점을 그린다. 계획 모드와 항법 모드의 카메라 동작이 다르다.
- `src/store.ts`는 재생 위치, 시나리오, 통신·시각 측위 상태와 인터페이스 상태를 관리한다.
- `src/nav.ts`는 WASM을 사용할 수 없을 때 쓰는 TypeScript A*, 경사 계산, 추정기를 포함한다. `src/nav.test.ts`에는 경로, 차단 구역 우회, 경사 대칭, 예측·관측, 각속도 통합을 검증하는 다섯 테스트가 있다.
- 웹의 이동, 통신 손실, 시각 측위 손실, 먼지, 장애물은 모두 클라이언트 측 시뮬레이션이다. 실제 센서, 위성 통신, 날씨 또는 궤도 데이터 피드는 연결돼 있지 않다.

## 알고리즘 상태

`mans-nav`의 A*는 격자 이동 비용과 고도 차이에 따른 경사 비용을 사용한다. `safest`일 때 23도 이상 경사를 제외하고, 현재는 테스트용 고정 차단 영역도 포함한다. 이는 임무 운용 경로 엔진으로 충분하지 않으며, 차단 영역을 데이터 기반 위험 레이어로 교체해야 한다.

상태 추정은 위치와 속도의 단순 예측 및 관측 보정이다. IMU, 비전, 태양 센서, 좌표계 변환, 시간 동기화, 불확실성 전파는 아직 구현되어 있지 않다.

## 재사용과 재구현 범위

| 구분 | 항목 | 조치 |
| --- | --- | --- |
| 직접 재사용 | `mans-nav`의 A*, 경사, 예측·관측과 Rust 테스트 | 네이티브 바이너리가 라이브러리를 직접 의존한다. |
| 유지 | 웹의 React UI, Three.js 씬, WASM 패키지, TypeScript 대체 경로 | 웹 데모의 기능과 빌드를 보존한다. |
| 적응 필요 | JSON 지형 계약, Rasterio 전처리 결과, 시나리오 상태 | 데이터 출처·좌표계·레이어 메타데이터를 포함하도록 확장한다. |
| 재구현 필요 | 네이티브 창 수명, 작업 공간, 3D 카메라, 데이터 카탈로그, 프로젝트 파일, 센서 인터페이스 | 웹 DOM 및 WASM 결합부를 사용하지 않고 Rust 경계에서 구현한다. |

## 주요 위험과 호환성

- 현재 경로 엔진의 고정 차단 사각형은 임무 데이터가 아니며 위험 레이어로 교체해야 한다.
- 샘플 지형의 로컬 격자에는 완전한 행성 좌표 변환 정보가 없다. 실제 DTM 도입 전에 투영과 원점을 보존해야 한다.
- `eframe`은 GPU 백엔드를 제공하지만 현재 초기 뷰포트는 검증용 2D 격자다. 3D 메시, 카메라, 청킹과 LOD는 후속 단계다.
- Apple Silicon에서 Cargo 빌드는 Command Line Tools를 가리키도록 `DEVELOPER_DIR`를 설정해 검증했다. 이 설정이 없는 주 Xcode 설치는 라이선스 동의 상태에 따라 빌드를 막을 수 있다.
- 현재 Rust 라이브러리는 `wasm-bindgen`도 의존한다. 네이티브 호출에는 문제 없지만, 장기적으로는 순수 코어와 웹 바인딩을 별도 crate로 분리하는 편이 바람직하다.

## 데이터 상태와 제한

`data/real/jezero-mola-128.json`에는 NASA MGS MOLA `MEGT44N000HB.IMG`에서 추출한 16 × 16 실제 관측 고도 창이 포함돼 있다. `tools/nasa-data/fetch_mola_jezero.py`가 같은 바이트 범위 추출을 재현하며 원본 PDS 제품과 메타데이터를 기록한다. 이 자료는 약 463 m 샘플링의 지역 규모 자료다. HiRISE DTM, 정사영상, 텍스처와 astronaut-scale 위험 자료는 아직 포함되지 않았다. `prepare_terrain.py`는 다음 제한을 가진다.

- Rasterio와 유효한 투영 좌표계가 필요하다.
- 미터 단위가 아닌 좌표계와 결측 화소를 거부한다.
- 현재 출력 크기와 셀 간격은 65 × 65, 12 m로 고정돼 있다.
- 스크립트는 고도 격자만 준비하며, 영상 텍스처, 위험도, 메타데이터 카탈로그를 만들지 않는다.

## 검증 기록

다음 명령은 이 작업 환경에서 성공했다.

```sh
npm run build
npm test
DEVELOPER_DIR=/Library/Developer/CommandLineTools /Users/paulkang/.cargo/bin/cargo test --manifest-path crates/mans-nav/Cargo.toml
DEVELOPER_DIR=/Library/Developer/CommandLineTools python3 -m py_compile data/prepare_terrain.py
DEVELOPER_DIR=/Library/Developer/CommandLineTools /Users/paulkang/.cargo/bin/cargo test --workspace
DEVELOPER_DIR=/Library/Developer/CommandLineTools /Users/paulkang/.cargo/bin/cargo run -p mans-desktop
```

웹 번들은 빌드되며 TypeScript 항법 테스트 5개와 Rust 단위 테스트 3개가 통과했다. `mans-desktop`은 Apple Silicon macOS에서 컴파일하고 창을 기동했다. 웹 번들에는 크기 경고가 남아 있다.

## 저장소 상태 확인 제한

이 호스트에서는 `git` 실행이 Xcode 라이선스 동의 요구로 중단된다. 이 전환 단계에서 브랜치 조작이나 Git 설정 변경은 하지 않았으며, 현재 브랜치와 작업 트리 상태는 확인하지 못했다.

