# PX4 고정익 TFLM 구현 계획

> 상태: 설계 초안 · 대상: 현재 보유한 고정익 기체 · 기준 문서: [PX4 TFLM 가이드](https://docs.px4.io/main/en/neural_networks/tflm)

## 0. 범위와 중요한 구분

PX4의 TFLM 가이드는 **멀티콥터용 실험 모듈 `mc_nn_control`**을 설명한다. 예제는 X500 V2에서 학습한 모델을 PX4 펌웨어에 포함하고, uORB에서 15개 `float` 입력을 만들어 추론한 뒤 4개 모터 명령을 `ActuatorMotors`로 발행한다. 이는 고정익에 바로 적용할 수 있는 모델이나 제어 인터페이스가 아니다. 고정익에서는 입력 피처, 모델, 출력 의미, 모듈 연결 지점, 시험 기준을 새로 정한다. [MC Neural Networks Control](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control), [TFLM 가이드](https://docs.px4.io/main/en/neural_networks/tflm)

**1차 범위:** PX4가 구동되는 고정익 FC 내부에 TFLM 추론 모듈을 추가하여 비행 상태/이상 징후를 **그림자 모드**로 계산·기록한다. 1차 결과로 서보·ESC를 구동하지 않는다. 모델이 효과를 입증하면 경고를 추가하고, 제어 연동은 별도 단계로 설계한다. FC 자원이 부족하면 별도 MCU 배치로 전환하되 동일한 데이터·모델·검증 계약을 유지한다. 이 선택은 [README.md](README.md)의 범용 구조를 PX4 내부 구현 중심으로 구체화한 것이다.

### 단계별 목표

| 단계 | 기능 | PX4와 연결 | 통과 조건 |
| --- | --- | --- | --- |
| A. 기준선 | 기존 PX4 고정익 설정·로그 확보 | 변경 없음 | 정상 비행 및 페일세이프 재현 |
| B. 추론 시험 | 고정 입력/기록 데이터로 TFLM 실행 | uORB 구독, 결과 로깅 | 모델 출력 일치, 자원 예산 충족 |
| C. 그림자 모드 | 실제 비행 데이터로 상태/이상 점수 계산 | 읽기 및 기록만 | 오경보·미탐·지연 기준 충족 |
| D. 경고 | 검증된 판정만 조종자에게 알림 | 별도 경고 경로 | 입력 오류 시 무효 처리와 경고 억제 |
| E. 제어 연구 | 제한된 상위 목표값 검토 | 별도 고정익 제어 인터페이스 | 시뮬레이션·실기체·조종권 복귀 시험 완료 |

## 1. 준비물

### 1.1 기체와 전자 장비

- [ ] 기체 형식(일반 고정익/비행익/VTOL), 총중량, 날개폭, 추진 방식, 정상 속도 범위, 운용 목적 기록.
- [ ] PX4 호환 FC의 **정확한 보드명·MCU·PX4 버전·빌드 타깃** 확인. 현재 펌웨어와 파라미터 백업. PX4가 아니면 PX4 이식 또는 별도 MCU 방식부터 결정.
- [ ] FC의 플래시·RAM 여유, CPU 부하, 스택 여유, 로그 저장 공간 및 전원 여유 확인. PX4 문서의 `mc_nn_control` 추가 크기 약 50 KB는 해당 예제의 참고값이며 고정익 모델의 예산이 아니다. [예제 빌드와 크기](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control)
- [ ] 필수 상태 계측 확인: IMU/자세·각속도, 위치·속도, 기압 고도, 비행 모드, 조종 입력, 배터리 전압·전류. 대기속도계가 있다면 종류·설치·보정·품질을 확인. GPS 지상속도를 대기속도와 동일시하지 않음.
- [ ] RC 송수신기와 수동 전환 스위치, 지상국(QGroundControl), 로그 수집 수단, 안전한 지상 전원 및 벤치 시험 환경 준비.
- [ ] 고정익 설정 기준선 확보: 조종면 방향·출력 매핑, 트림, 자세/속도 제어, 대기속도 범위, Return/링크 손실/배터리 페일세이프. [고정익 초기 설정](https://docs.px4.io/main/en/config_fw/index), [페일세이프](https://docs.px4.io/main/en/config/safety)

### 1.2 개발 환경과 자료

- [ ] PX4 소스와 하위 모듈을 특정 **릴리스/커밋으로 고정**하고, 그 버전의 문서·보드 설정·uORB 메시지와 맞춰 작업. `main` 문서는 변경될 수 있다.
- [ ] PX4 빌드 환경, ARM 교차 컴파일러, CMake/Ninja, Python 가상환경, QGroundControl, Gazebo 설치. PX4 예제의 neural 타깃은 **Ubuntu 24.04 이상**을 요구하므로 우선 해당 환경 또는 그에 맞는 공식 개발 컨테이너를 확보한다. 일반 PX4 개발의 macOS 지원과 예제 neural 빌드 지원을 구별한다. [neural 빌드 조건](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control), [개발 환경](https://docs.px4.io/main/en/dev_setup/dev_env), [PX4 개발 컨테이너](https://docs.px4.io/main/en/test_and_ci/docker)
- [ ] TensorFlow/TFLM 변환 환경, 모델 구조·학습 코드, 데이터 변환 스크립트, `.tflite` 검사 및 C 배열 생성 도구(`xxd`) 준비.
- [ ] 실제 기체 ULog 또는 기존 정상 비행 기록, 비행 당시 파라미터, 센서 보정 정보, 환경 정보 확보. 없으면 기존 PX4만으로 기준 비행부터 실시.
- [ ] 비교 대상인 규칙 기반 또는 물리 기반 판정 방법 정의. 데이터가 부족한 경우 학습보다 계측과 로그 정리에 우선순위를 둔다.

### 1.3 설계 착수 전에 확정할 값

| 항목 | 결정할 내용 | 현재 상태 |
| --- | --- | --- |
| 첫 추론 문제 | 비행 상태 분류/이상 징후 중 **한 가지**와 사용자가 취할 행동 | 미정 |
| 입력 계약 | uORB 토픽, 필드, 단위, 좌표계, 센서 유효성, 표본 주기, 시간 창 | 미정 |
| 출력 계약 | 점수/클래스 의미, 임계값, 유효 시간, 지속 확인, 무효 상태 | 미정 |
| 실시간 예산 | 추론 주기, 데이터 최대 나이, 전처리·Invoke 최악 시간, CPU/메모리 상한 | 보드 실측 후 결정 |
| 평가 기준 | 비행시간당 오경보, 이상 탐지율, 탐지 선행 시간, 조건별 성능 | 데이터 검토 후 결정 |
| 배치 | FC 내부 모듈 또는 별도 MCU, 전원·링크·로그 경로 | FC 내부 우선 검토 |

## 2. 구현 준비

### 2.1 PX4 고정익 기준선 확인

1. 대상 기체를 PX4 기본 펌웨어로 설정·보정하고 트림 및 기존 제어를 정상화한다. PX4의 고정익 계층은 버전에 따라 이름이 바뀔 수 있으므로 고정한 버전에서 실제 모듈을 확인한다. 현행 문서에는 `fw_mode_manager`(내부 모드의 setpoint 생성), `fw_lat_lon_control`(횡·종 제어), `fw_att_control`, `fw_rate_control`이 설명되어 있다. [PX4 Controller Modules](https://docs.px4.io/main/en/modules/modules_controller)
2. 고도·대기속도 제어의 TECS와 횡방향 경로 제어의 NPFG를 포함하여 현 기체의 제어 성능을 먼저 기록한다. [고정익 위치·고도 제어](https://docs.px4.io/main/en/config_fw/position_tuning_guide_fixedwing)
3. `make px4_sitl gz_rc_cessna` 또는 `make px4_sitl gz_advanced_plane`으로 고정익 SITL 기준 시나리오를 만든다. 예제 `make px4_sitl_neural`은 멀티콥터 신경망 타깃의 빌드 확인용이며 고정익 기능 검증을 대신하지 않는다. [Gazebo 고정익 기체](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)
4. 실제 기체와 시뮬레이터의 차이(중량, 관성, 양력·항력, 추진, 센서 지연)를 문서화한다. 비행 데이터로 모델을 평가할 때 해당 차이를 조건별로 분리한다.

### 2.2 데이터·모델 계약 수립

- 각 피처의 **출처 uORB 토픽/필드, 좌표계, 단위, 정상 범위, 결측 처리**를 표로 고정한다. 학습 코드와 PX4 모듈이 같은 순서와 전처리 상수를 써야 한다. PX4 예제도 시뮬레이터 ENU와 PX4 NED 사이의 좌표 변환을 명시적으로 수행한다. [예제 입력 변환](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control)
- 예시 후보 피처: 대기속도 및 유효 플래그, 고도/수직속도, roll·pitch, 각속도, 스로틀 명령, 배터리 전압·전류. 실제 모델 입력에는 첫 기능과 로그 품질을 검토한 신호만 넣는다. 받음각 센서가 없다면 받음각을 측정값으로 가정하지 않는다.
- 타임스탬프를 기준으로 신호를 정렬하고 오래된 값, NaN/Inf, 보정 중 센서, 비행 모드 전환 구간을 처리한다. 데이터가 유효하지 않으면 추론 출력을 `invalid`로 기록하고 경고로 연결하지 않는다.
- 비행/날짜/기체 단위로 학습·검증·독립 시험 데이터를 분리한다. 겹치는 시간 창이 양쪽에 섞이지 않도록 한다. 이상 사건의 레이블은 원인과 확인 근거를 기록한다.
- 작은 기준 모델을 학습하고 `.tflite`로 변환한다. 모델의 지원 연산자와 텐서 타입을 검사하고, 양자화는 모델 및 보드 성능을 비교한 후 선택한다. PX4 예제의 `float32` 15→4 형식은 **해당 예제의 시작 검사**이며 새 고정익 모듈의 요구사항이 아니다. [TFLM 모델 규격과 검사](https://docs.px4.io/main/en/neural_networks/tflm)

### 2.3 구현 경로 선택 게이트

| 선택지 | 채택 조건 | 작업 영향 |
| --- | --- | --- |
| **PX4 FC 내부 TFLM 모듈** | 보드에 플래시·RAM·CPU/스케줄러 여유가 있고 TFLM 연산자 지원 확인 | PX4 소스 빌드, uORB 구독, 모듈·보드 설정 추가 |
| 별도 추론 MCU | FC 여유 부족, 추론 격리 필요, FC 데이터 링크 확보 | MCU 펌웨어와 통신 프로토콜·시간 동기화 구현 |

1차 계획은 **FC 내부 모듈**로 진행한다. 보드 실측에서 예산을 넘거나 고정익 제어 주기에 간섭하면 별도 MCU 경로로 전환한다. `mc_nn_control` 전체를 켜서 고정익에 적용하는 방식은 사용하지 않는다.

## 3. 실제 구현 항목: FC 내부 모듈

### 3.0 PX4 소스에서 바꿀 파일

아래 경로는 **현재 PX4 `main`의 실제 소스 구조를 확인한 작업 목록**이다. 구현 전에 선택한 릴리스/커밋에서 경로와 설정 이름을 다시 확인한다. 1차 모듈 이름은 `fw_nn_monitor`로 정하고, 그림자 모드의 관측 기능만 구현한다.

| PX4 경로 | 변경 내용 | 1차 필요 여부 |
| --- | --- | --- |
| `src/modules/fw_nn_monitor/Kconfig` | `MODULES_FW_NN_MONITOR` 빌드 선택 항목 신설 | 필수 |
| `src/modules/fw_nn_monitor/CMakeLists.txt` | `px4_add_module()`로 소스·파라미터 등록, `tensorflow_lite_micro`와 필요한 PX4 라이브러리 연결 | 필수 |
| `src/modules/fw_nn_monitor/fw_nn_monitor.cpp/.hpp` | uORB 구독, 전처리, 모델 검사·초기화·추론, 출력 유효성 검사, 상태 보고 | 필수 |
| `src/modules/fw_nn_monitor/model_data.cpp/.hpp` | 검증된 `.tflite`를 포함한 C 배열과 크기·버전 | 필수 |
| `src/modules/fw_nn_monitor/fw_nn_monitor_params.yaml` | 활성화 플래그(초기값 비활성), 추론 주기 등 필요한 설정 | 필수 |
| `boards/<제조사>/<보드>/<설정>.px4board` | `CONFIG_LIB_TFLM=y`, `CONFIG_MODULES_FW_NN_MONITOR=y` 추가; 기존 고정익 필수 모듈 유지 | 필수 |
| `ROMFS/px4fmu_common/init.d/rc.fw_apps` | 활성화 파라미터와 모듈 포함 여부를 확인한 뒤 `fw_nn_monitor start` 실행 | 자동 시작 시 필수 |
| `msg/FwNnMonitor.msg`, `msg/CMakeLists.txt` | 입력 시각·점수·유효성·오류·모델 버전·추론 시간용 uORB 메시지 정의·등록 | 전용 결과 기록 시 필수 |
| `src/modules/logger/logged_topics.cpp` | 새 토픽을 logger의 적절한 프로필에 등록 | ULog 기록 시 필수 |

PX4 예제의 [`src/modules/mc_nn_control/CMakeLists.txt`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_nn_control/CMakeLists.txt), [`Kconfig`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_nn_control/Kconfig), [`rc.mc_apps`](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rc.mc_apps), [`NeuralControl.msg`](https://github.com/PX4/PX4-Autopilot/blob/main/msg/NeuralControl.msg), [logger 등록](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/logger/logged_topics.cpp)을 구현 패턴으로 참고한다. 고정익 시작 경로는 [`rc.fw_apps`](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rc.fw_apps)이다. 현재 PX4의 `src/modules/Kconfig`는 하위 모듈의 `Kconfig`를 읽으므로 새 모듈의 Kconfig 파일을 만든다. [PX4 소스](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/Kconfig)

**복사하면 안 되는 설정:** 예제 [`boards/px4/fmu-v6c/neural.px4board`](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6c/neural.px4board)는 `CONFIG_LIB_TFLM=y`와 `CONFIG_MODULES_MC_NN_CONTROL=y`를 포함하지만 `CONFIG_MODULES_FW_ATT_CONTROL=n`, `CONFIG_MODULES_FW_MODE_MANAGER=n`, `CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL=n`, `CONFIG_MODULES_FW_RATE_CONTROL=n`으로 고정익 제어를 비활성화한다. 고정익에서는 대상 보드의 정상 설정을 출발점으로 삼고, 필수 FW 모듈을 유지한 채 TFLM을 추가한다. [기본 보드 설정 예](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6c/default.px4board)

**1차에 수정하지 않을 코드:** `fw_mode_manager`, `fw_lat_lon_control`, `fw_att_control`, `fw_rate_control`, `control_allocator`, `commander`의 제어 로직은 그림자 모드와 경고를 위해 바꾸지 않는다. `mc_nn_control`의 모터 명령 발행 코드도 사용하지 않는다. TFLM 라이브러리는 PX4의 [`src/lib/tensorflow_lite_micro/`](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/tensorflow_lite_micro)에 통합되어 있으므로, 고정한 버전에서 모델 연산자가 지원된다면 라이브러리 자체를 수정할 필요가 없다. 제어 연동을 시작할 때만 고정익의 setpoint 소유권·외부 모드 등록·전환·페일세이프에 필요한 변경을 별도로 결정한다.

### 3.1 PX4 빌드 통합

- [ ] 고정한 PX4 소스에서 TFLM 라이브러리 빌드 옵션과 대상 보드 설정을 확인한다. 보드의 정상 고정익 설정에 `CONFIG_LIB_TFLM=y`, `CONFIG_MODULES_FW_NN_MONITOR=y`를 추가하고 펌웨어 크기를 계측한다. [예제 빌드 설정](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control)
- [ ] `src/modules/fw_nn_monitor/`의 `Kconfig`, `CMakeLists.txt`, 모듈 소스, 모델 데이터, 파라미터를 만들고 해당 버전의 PX4 빌드 방식에 연결한다. 초기 기능은 일반 비행 모드와 독립적으로 상태를 관측하는 태스크로 둔다.
- [ ] 부팅 시 자동 시작할 경우 활성화 파라미터와 `rc.fw_apps`의 시작 조건을 추가한다. 활성화 파라미터는 시험 전까지 기본값을 꺼 둔다. 명령으로 시작·중지·상태 확인도 가능하게 한다. [예제 자동 시작 구조](https://docs.px4.io/main/en/neural_networks/nn_module_utilities)

### 3.2 모델 포함 및 TFLM 실행

- [ ] 검증된 `.tflite`를 C/C++ 바이트 배열로 변환한다. PX4 가이드의 `xxd -i converted_model.tflite > model_data.cc`는 출발점이다. 생성 배열과 길이를 헤더에서 선언하고 모델 파일 해시·버전을 기록한다. [모델 포함 방법](https://docs.px4.io/main/en/neural_networks/tflm)
- [ ] 모델 스키마 버전, 입력·출력 **개수·shape·type·양자화 scale/zero point**를 시작 시 검사한다. 불일치 시 모듈을 비활성화하고 원인을 로그에 남긴다. 새 입력 차원과 출력 차원은 기능 계약에서 결정한다.
- [ ] 실제 모델의 연산자만 `MicroMutableOpResolver`에 등록하고 `MicroInterpreter`, 정렬된 `tensor_arena`, 텐서 할당을 초기화한다. 추론 실패 시 다음 주기에 오래된 출력을 재사용하지 않는다. [PX4 예제의 resolver·interpreter](https://docs.px4.io/main/en/neural_networks/tflm)
- [ ] 양자화 모델을 택하면 학습 파이프라인과 펌웨어의 quantize/dequantize 처리를 일치시킨다. PC 원본 모델, 변환 모델, FC 실행 결과를 동일한 입력 기록으로 비교한다.
- [ ] arena·스택·입력 버퍼와 펌웨어 전체 크기를 계측하고, 실제 FC에서 `Invoke()` 지연의 평균·상위 지연·최악 지연을 측정한다. [TFLM 메모리 구조](https://github.com/tensorflow/tflite-micro/blob/main/tensorflow/lite/micro/docs/online_memory_allocation_overview.md)

### 3.3 고정익 데이터 연결과 결과 처리

- [ ] 선택한 uORB 토픽을 구독하고 타임스탬프 기준으로 입력을 구성한다. 좌표계·단위 변환은 독립 함수로 만들고 학습 데이터 변환과 같은 시험 벡터로 확인한다. [uORB 메시징](https://docs.px4.io/main/en/middleware/uorb)
- [ ] 필터·정규화·시간 창·결측 처리를 입력 계약 그대로 구현한다. 비행 모드나 센서 유효성이 바뀌면 시간 창 초기화/무효 처리 규칙을 적용한다.
- [ ] 출력은 `score`, `valid`, `reason`, `model_version`, 입력 시각, 추론 소요시간 등의 **관측 결과**로 발행한다. 필요하면 전용 uORB 토픽을 정의하고 logger에 포함한다. PX4 예제도 전용 `NeuralControl` uORB 메시지와 logger 설정을 사용한다. [예제 로깅](https://docs.px4.io/main/en/neural_networks/nn_module_utilities)
- [ ] 경고 단계에서는 범위, 입력 신선도, 연속 확인, 히스테리시스/해제 조건을 통과한 결과만 지상국에 알린다. 비활성·오류·유효하지 않은 입력은 제어 명령으로 바뀌지 않는다.

### 3.4 나중의 제어 연동 설계 항목

고정익 제어를 목표로 한다면 **별도 설계 검토**를 수행한다. 제어할 대상을 경로/고도/대기속도 등의 상위 setpoint 중 하나로 정하고, 해당 PX4 버전의 `fw_mode_manager` → `fw_lat_lon_control` → 자세/각속도 제어 흐름에서 소유권과 uORB 연결 지점을 확인한다. 기존 제어기와 같은 토픽을 동시에 발행하지 않도록 모드 전환과 발행 권한을 정의한다. PX4 예제의 외부 모드 등록(`RegisterExtComponentRequest`, arming check, `VehicleStatus`)은 참고 구조지만, `ActuatorMotors`로 4개 모터 힘을 발행하고 control allocation을 끄는 방식을 고정익에 복사하지 않는다. [PX4 NN 통합](https://docs.px4.io/main/en/neural_networks/nn_module_utilities), [고정익 컨트롤러](https://docs.px4.io/main/en/modules/modules_controller)

제어 단계의 추가 조건: 조종자 우선권, 입력·출력 상한, rate limit, 최소 대기속도/고도 등 비행 영역 제한, 추론 타임아웃, 모델 이상 시 기존 모드로 복귀, 통신·센서 장애 대응, 페일세이프 우선순위. 해당 조건을 검증하기 전에는 NN 출력이 액추에이터 경로에 도달하지 않는다.

## 4. 시험과 수용 기준

| 순서 | 시험 | 확인할 결과 |
| --- | --- | --- |
| 1 | 데이터 계약 시험 | 기록 입력과 펌웨어 입력의 값·순서·단위·좌표계 일치 |
| 2 | 모델 교차 시험 | Python 원본, `.tflite`, FC TFLM의 동일 입력 출력 차이가 정한 허용 범위 이내 |
| 3 | 자원 시험 | 펌웨어 크기, arena/RAM, 스택, CPU 부하, 추론 최악 시간이 실측 예산 이내 |
| 4 | 고정익 SITL에서 `gz_rc_cessna`/`gz_advanced_plane`의 정상·센서 누락·지연·모드 전환·재시작 | 기존 비행 경로와 페일세이프 방해 없음 |
| 5 | 실기체 지상시험: 전원·부팅·로그·링크 단절, 비활성화/복구, 수동 조종 확인 | 모듈 실패가 FC 기본 동작에 영향 없음 |
| 6 | 그림자 모드 비행: 다양한 속도·고도·바람·배터리 조건에서 추론만 기록 | 사전 정의한 오경보·미탐·지연 기준 충족 |
| 7 | 경고 비행: 조종자에게 경고만 제공하고 입력 오류를 주입 | 유효하지 않은 결과는 경고하지 않음 |
| 8 | 별도 제어 시험: 고정익 전용 제어 설계가 승인된 뒤 SITL→보드→제한된 비행 | 조종권 복귀·페일세이프·비행 영역 제한 검증 |

시험은 **모델 버전 + PX4 커밋 + 보드 타깃 + 기체 파라미터 + 데이터셋 버전**을 한 묶음으로 보관한다. 수치 허용치는 첫 기능, 기체, 센서 품질과 보드 계측이 끝난 뒤 적는다. 모르는 임계값을 임의로 고정하지 않는다.

## 5. 작업 순서와 산출물

| 순번 | 작업 묶음 | 산출물 | 다음 단계 조건 |
| --- | --- | --- | --- |
| P0 | 기체·FC 조사와 PX4 버전 고정 | 기체 제원표, 보드 자원표, 펌웨어/파라미터 백업 | 현재 고정익 비행 구성이 재현됨 |
| P1 | 고정익 SITL 및 실제 기준 로그 | 재현 명령, 정상 비행 ULog, 페일세이프 기록 | 입력 후보가 실제로 수집됨 |
| P2 | 첫 기능·입출력·성능 기준 확정 | 피처/출력 계약서, 데이터 분할표, 기준선 | 학습·평가 가능 |
| P3 | 학습·변환·PC 교차 검증 | 학습 코드, `.tflite`, 변환 로그, 시험 벡터 | 모델 형식과 출력 확인 |
| P4 | PX4 TFLM 모듈과 로깅 | 고정익 모듈 소스, 보드 빌드, 로그 토픽 | SITL·실보드에서 자원 예산 충족 |
| P5 | 장애 시험과 그림자 모드 비행 | 시험 보고서, 비행 로그, 오경보·미탐 분석 | 경고 단계 진행 여부 결정 |
| P6 | 경고, 필요 시 별도 제어 연구 | 경고 운용 절차 또는 고정익 제어 설계서 | 안전·성능 게이트 통과 |

## 6. 현재 미확정 사항

실제 구현 착수 시 **기체 모델, FC 보드/현재 PX4 버전, 대기속도계 유무, 기존 ULog, 첫 목표 기능**을 먼저 채운다. 이 정보 없이 특정 입력 차원, 모델 구조, arena 크기, 주기, 보드 빌드 명령, 출력 토픽을 확정하지 않는다.

### 핵심 참고 문서

- [PX4: TensorFlow Lite Micro 통합](https://docs.px4.io/main/en/neural_networks/tflm)
- [PX4: 멀티콥터 NN 예제의 범위와 빌드](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control)
- [PX4: NN 모듈 자동 시작·외부 모드·로깅](https://docs.px4.io/main/en/neural_networks/nn_module_utilities)
- [PX4: 고정익 컨트롤러 모듈](https://docs.px4.io/main/en/modules/modules_controller)
- [PX4: 고정익 Gazebo 모델](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)
