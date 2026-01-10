포트폴리오형이 좋고 심작동센서에 사용된 필터도 언급했으면 좋을 것 같아 왜 그 필터를 사용했는지

아래는 **포트폴리오형 README 문안**으로, 센서 모델명은 쓰지 않고(온습도/공기질/심박동 센서로만 표기), **코드/파라미터는 직접 노출하지 않으며**, 자료에 나온 기술과 효과를 중심으로 정리했습니다 [1][2][3].  
심박동 센서에는 자료에 명시된 **HPF + LPF(대역통과 형태)** 필터를 “왜 썼는지”까지 포함했습니다 [3][2].

***

# Health Management (Sensor Fusion, Risk Reasoning)

운동/활동 중 **열 환경(온도·습도)**, **공기질(가스 농도 PPM)**, **심박 신호**를 동시에 수집하고, **센서 노이즈를 억제한 뒤 위험도를 상황별로 판단**하여 “휴식/열 관리/호흡 관리/정상”을 안내하는 실시간 헬스 모니터링 시스템입니다 [1][3].  
단순 임계값 경보가 아니라, **센서 파싱 → EKF 기반 상태추정 → 3-Way Heuristics(Trend/Pattern/Spike) → Adaptive Threshold**로 이어지는 “알고리즘 중심 파이프라인”을 설계한 것이 핵심입니다 [1][3].

## 입력 신호 (범용 센서 표기)

- **온·습도 센서**: 온도/습도를 기반으로 **Heat Index(체감온도)**를 산출하고 열 위험 평가에 사용합니다 [1][3].  
- **공기질 센서**: 아날로그 신호를 **가스 농도(PPM)**로 정량화하여 호흡 위험 지표로 사용합니다 [1][3].  
- **심박동 센서**: 아날로그 심박 파형을 필터링해 변동을 줄이고 안정적인 입력으로 사용합니다 [2][3].

## 핵심 기술 1: 센서 파싱(정량화) + 상태 표현

이 시스템은 “Raw 센서값”을 그대로 쓰지 않고, 각 신호를 **의미 있는 단위/지표**로 바꾼 뒤 알고리즘에 투입합니다 [1][3].  
온·습도는 **Heat Index Trend(체감온도 추세)**로 표현되어 열 환경이 “좋아지는지/나빠지는지”를 연속적으로 해석할 수 있게 구성되어 있습니다 [1].  
공기질은 PPM 단위의 농도 추세로 관리되며, 열 지표와 함께 위험도를 분리해 판단할 수 있도록 설계되어 있습니다 [1][3].

## 핵심 기술 2: EKF 기반 센서 융합(노이즈 억제)

자료에서는 **EKF(확장 칼만 필터)**의 예측–갱신 구조를 통해, 측정치의 흔들림을 줄이고 “현재 상태(온도/습도/공기질)”를 안정적으로 추정하는 과정을 핵심 요소로 제시합니다 [1].  
실제 로그에서도 Raw 값과 EKF 추정값이 함께 제시되며, EKF가 측정 노이즈를 누르고 더 안정적인 추정치로 수렴하는 흐름이 확인됩니다 [1].  
이 설계의 효과는, 위험 판단이 순간적인 잡음에 휘둘리지 않고 **상태 변화의 방향성과 크기**를 기반으로 동작하도록 만든다는 점입니다 [1][3].

## 핵심 기술 3: 심박 신호 필터링(HPF + LPF)

심박 신호는 센서 특성상 **저주파 드리프트(기준선 흔들림)**와 **고주파 잡음**이 섞이기 쉬워 “그대로 쓰면” 상태 판정에 불리합니다 [3].  
자료/코드에는 심박 신호를 **High-pass + Low-pass(대역통과 형태)**로 처리하는 구조가 명시되어 있으며, 이는 느린 추세성 흔들림(DC/기준선 성분)을 줄이고, 너무 빠른 잡음을 억제해 **심박 파형의 유효 대역**을 남기려는 목적의 설계로 해석됩니다 [3][2].  
즉, 심박 입력을 “측정 노이즈가 큰 신호”에서 “판단 가능한 신호”로 만드는 전처리 단계로서 필터를 사용한 것이 포인트입니다 [3].

## 핵심 기술 4: 3-Way Heuristics (시나리오 기반 위험 탐지)

자료의 위험 판단은 하나의 룰이 아니라, 아래 **3가지 시나리오**를 병렬로 고려하는 구조입니다 [1][3].

- **Trend Risk(추세 위험)**: 일정 구간의 변화량을 보고 “점진적으로 악화되는 상황”을 탐지합니다 [1][3].  
- **Pattern Risk(패턴 위험)**: 표준편차(SD) 기반으로 “평소 대비 변동성이 커진 이상 패턴”을 탐지합니다 [1][3].  
- **Spike Risk(스파이크 위험)**: Raw와 추정값의 차이를 이용해 “순간 급변”을 위험 신호로 탐지합니다 [1][3].

이 구조의 장점은 **서서히 나빠지는 케이스(Trend)**와 **불안정해지는 케이스(Pattern)**, **순간적으로 튀는 케이스(Spike)**가 서로 다른 방식으로 나타나더라도 놓치지 않도록 설계했다는 점입니다 [1][3].

## 핵심 기술 5: Adaptive Threshold (LTSD 기반 동적 임계값)

패턴 위험 탐지에서 중요한 부분은, SD를 고정 임계값으로 비교하지 않고 **LTSD(Long-Term Standard Deviation)를 EMA로 갱신**해 “평소 변동성”을 기준으로 임계값이 적응하도록 만든 점입니다 [1].  
자료에는 LTSD가 “historical SD의 EMA”로 정의되고, 이를 이용해 **Adaptive SD Threshold**를 구성하는 개념이 제시됩니다 [1].  
이 방식은 환경 변화나 센서 품질 차이로 변동폭이 달라져도, 경보가 과도하게 울리지 않도록 **안정적인 위험 탐지**를 목표로 합니다 [1][3].

## 출력(의사결정)과 효과

시스템은 센서 분석 결과를 사용자 행동으로 연결하기 위해, 결과를 **OPTIMAL / MANAGE HEAT / MANAGE BREATH / TAKE A BREAK** 형태의 명확한 메시지로 제공합니다 [1][3].  
또한 Heat Index Trend와 Gas Concentration(PPM)을 함께 보여주는 구성으로, 열 환경과 공기질 위험을 분리해서 이해할 수 있도록 설계되어 있습니다 [1].  
서버 측에서도 수신 데이터(온도·습도·가스·Heat Index·예측값·Intensity)를 로깅하는 흐름이 제시되어, “엣지에서 판단한 결과”를 서비스/대시보드로 확장할 수 있는 형태를 갖추고 있습니다 [1][2].

***

원하면 마지막에 포트폴리오에서 강하게 먹히는 **“My Contribution(담당 업무)”** 섹션을 5~6줄로 더 붙여줄게요. (예: *센서 파싱 설계 / EKF 융합 구조 구현 / 3-Way 시나리오 판정 로직 설계 / LTSD 기반 적응형 임계값 도입 / 심박 신호 필터링 설계*)

인용:
[1] helseu_maenijimeonteu_siseutem.pptx.pdf https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/fdec0bfc-721d-4129-8ccc-8ba71eabc168/helseu_maenijimeonteu_siseutem.pptx.pdf
[2] aduino_coejongbalpyo.pptx.pdf https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/9de2a5d9-7d3a-41f2-acf4-3e9039adefba/aduino_coejongbalpyo.pptx.pdf
[3] HealthManagement.ino https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/41682a96-6e54-4f0a-99f9-efc041221970/HealthManagement.ino
[4] SimpleWebServerWiFi.ino https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/82e46fe6-d56b-4566-9ba8-1d62195e37d5/SimpleWebServerWiFi.ino
[5] train.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/c3886d44-5e06-4fbe-9d32-a254caaadac9/train.py
[6] val.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/9ad764b6-cc06-4340-ba7b-36069bec536f/val.py
[7] datasets.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/5015a7b5-746b-4f57-8c73-c36e65ea3c67/datasets.py
[8] kobert_val.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/816b6018-789c-4770-a088-51f050ea34a6/kobert_val.py
[9] kobert_execution.bash https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/5ade8d4a-b6c7-4d8f-bf4c-b55a3325117c/kobert_execution.bash
[10] kobert_question.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/a7cc70cf-9a2e-4821-b612-1b8d54e08451/kobert_question.py
[11] kobert_train.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/170a4adf-a244-4ff0-940d-8b5cc4771686/kobert_train.py
[12] kobert_result.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/af4d7418-3ad3-4011-bdaf-06f33af961d1/kobert_result.py
[13] kaebseuton-gyehoegseo.pdf https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/d01cc2f7-c6ca-4dbf-8f7d-1d126dba305e/kaebseuton-gyehoegseo.pdf
[14] crack_val_risk.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/d74ec14c-7154-44cb-8ac2-e0bb33d05b15/crack_val_risk.py
[15] crack_val_multi_analysis.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/5f446396-9b6a-4e28-91c6-136296799518/crack_val_multi_analysis.py
[16] crack_val_analysis.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/a98c628f-d379-489c-af62-bf40c5776ab4/crack_val_analysis.py
[17] crack_val_prediction.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/e93d634e-c1b0-4a3e-8ef7-210ce29105a0/crack_val_prediction.py
[18] crack_val_video.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/4f398eb9-fe02-48fd-9592-3f578de7d236/crack_val_video.py
[19] crack_val_angel.py https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/c4528964-48ab-4a81-8915-e1576b05b50c/crack_val_angel.py
[20] keompyuteo_bijeon.pptx.pdf https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/71906464/46233b3b-5776-4147-bf72-d138887e810d/keompyuteo_bijeon.pptx.pdf
