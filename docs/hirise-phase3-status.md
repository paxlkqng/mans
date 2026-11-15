# HiRISE 고해상도 지형 단계 상태

## 확인된 원본

- DTM: `DTEEC_045994_1985_046060_1985_U01.IMG`
- 정사영상: `ESP_045994_1985_RED_C_01_ORTHO.JP2`
- 지역: Jezero Crater, Mars
- 제공처: University of Arizona HiRISE PDS archive
- 원본 크기: DTM 약 381 MB, 정사영상 약 34 MB

두 제품의 정확한 원본 URL과 처리 조건은 `tools/nasa-data/hirise-jezero-045994-046060.json`에 기록했다.

## 구현됨

- 원본 제품 식별자와 URL이 버전 관리되는 매니페스트에 기록돼 있다.
- `extract_hirise_subset.py`는 로컬 원본에서 작은 DTM 및 정사영상 윈도우를 추출한다.
- 추출기는 범위, 겹침, DTM 결측, 영상 결측, 정합된 격자 크기를 검사한다.
- 결과에는 CRS, affine transform, 해상도, 원본 파일명과 처리 방법을 기록한다.
- MOLA 지역 참조층과 네이티브 Rust A* 경로는 유지된다.

## 아직 구현되지 않음

- HiRISE 원본의 로컬 다운로드와 검증된 Jezero 윈도우 선택
- 추출된 GeoTIFF의 네이티브 Rust 로더
- wgpu 기반 3D 메시, 텍스처 업로드, 카메라 조작, 3D 피킹
- HiRISE 표면 위의 경로 렌더링과 측정 도구

따라서 현재 네이티브 앱은 실제 MOLA 관측 격자를 사용하는 2D 작업 표면이며, HiRISE 3D 지형을 표시한다고 주장하지 않는다. HiRISE 원본을 내려받고 GDAL/Rasterio 환경에서 추출기를 실행한 뒤, 생성한 `elevation.tif`, `ortho.tif`, `provenance.json`을 Rust terrain crate로 연결하는 것이 다음 필수 단계다.
