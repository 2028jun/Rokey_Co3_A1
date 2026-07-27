# 프롬프트: 식당 좌/우 벽에 "넓어 보이는" 사진 벽화 붙이기

Isaac Sim이 있는 GPU 컴퓨터에서 이 파일을 그대로 코딩 에이전트에게 붙여넣어서 실행할 것.

## 목표

`nav_robot/assets/lightweight_restaurant/lightweight_pizza_restaurant.usda`의
`LeftWall` / `RightWall` (Architecture 그룹, 현재 웨인스코팅+트림+상단 3단 구조로
분리되어 있음)에, **지금 씬에 실제로 있는 테이블/의자를 그대로 촬영한 사진**을
텍스처로 붙여서, 마치 식당이 실제보다 2~3배 넓게 이어지는 것처럼 보이게 만든다.

이미지 파일도 거울 재질도 아니고, **실제 씬을 찍은 스크린샷을 벽에 붙이는 방식**이다
— 그래야 현재 테이블 스타일·조명·바닥색이랑 이질감 없이 자연스럽게 이어져 보인다.

## 1단계 — 씬 촬영

1. Isaac Sim에서 식당 씬을 연다 (`nav_restaurant_demo.py`를 로봇 없이 띄우거나,
   `lightweight_pizza_restaurant.usda`를 직접 열어도 됨).
2. 씬 끝에 이미 `def Camera "Camera"`가 정의되어 있다 — 이 카메라 뷰를 참고하거나,
   테이블 줄이 벽 쪽으로 쭉 이어져 보이는 각도로 새 카메라/뷰포트를 잡는다.
   - 벽과 거의 평행하게, 테이블 1~2개 줄이 그 벽 방향으로 멀어지며 보이도록 구도를 잡을 것
     (원근감이 있어야 "더 이어지는" 느낌이 남).
   - 바닥·천장(또는 위쪽 여백)까지 프레임에 들어가게 촬영해서, 나중에 벽 상단 밴드
     (`LeftWall_Upper` / `RightWall_Upper`, 현재 크림톤 `(0.9, 0.85, 0.74)`) 영역에
     자연스럽게 맞출 수 있게 한다.
3. 뷰포트 캡처 (`Viewport` → `Capture Screenshot` 또는 `omni.kit.viewport.utility`의
   캡처 API)로 최소 1920×1080 이상 해상도로 PNG 저장.
4. 좌/우 벽 각각 한 장씩 촬영 (같은 이미지를 좌우 반전해서 재사용해도 되지만, 실제로
   따로 찍으면 더 자연스러움).
5. 저장 위치:
   - `nav_robot/assets/lightweight_restaurant/textures/dining_extension_left.png`
   - `nav_robot/assets/lightweight_restaurant/textures/dining_extension_right.png`

## 2단계 — 이미지 보정 (선택, 하지만 강력 권장)

- 이미지의 위/아래/안쪽(방 쪽) 가장자리를 벽 색(크림톤 `(0.9, 0.85, 0.74)`)으로
  살짝 페더/블러 처리해서 사진과 실제 벽 사이 경계선이 안 보이게 한다 — 안 하면
  "사진이 벽에 붙어있다"는 티가 확 남.
- 원근감이 있는 사진이라면 그대로 써도 되지만, 벽 전체를 덮을 만큼 평평하게
  늘려야 하면 원근 왜곡이 생길 수 있음 — 이 경우 촬영 각도를 벽과 더 평행하게
  다시 잡는 게 이미지 왜곡 보정보다 낫다.

## 3단계 — USD 머티리얼로 벽에 입히기

`LeftWall_Upper` / `RightWall_Upper` Cube는 큐브라 UV가 6면에 반복 매핑되어
사진을 붙이기 부적합하다. **Cube 대신 안쪽을 향하는 얇은 Plane을 벽 앞에
덧대고, 그 Plane에만 사진 텍스처를 입힌다** (기존 Cube는 그대로 두고 색을
가려도 되고, Plane이 딱 앞에 오므로 안 보여도 됨).

`lightweight_pizza_restaurant.usda`의 `Architecture` 그룹 안, `LeftWall_Upper`
정의 바로 뒤에 아래를 추가 (RightWall도 동일 패턴, 좌표만 반대):

```usda
def Mesh "LeftWall_Mural" (
    prepend apiSchemas = ["MaterialBindingAPI"]
)
{
    # 벽면(-6, y, z)에서 안쪽(+x 방향)으로 살짝 띄워서 z-fighting 방지
    point3f[] points = [
        (-5.98, -4.6, 1.08), (-5.98, 4.6, 1.08),
        (-5.98, 4.6, 3.0), (-5.98, -4.6, 3.0)
    ]
    int[] faceVertexCounts = [4]
    int[] faceVertexIndices = [0, 1, 2, 3]
    normal3f[] normals = [(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 0)]
    texCoord2f[] primvars:st = [(0, 0), (1, 0), (1, 1), (0, 1)] (
        interpolation = "vertex"
    )
    uniform token subsetFamily:materialBind:familyType = "nonOverlapping"
    rel material:binding = </World/Architecture/Looks/LeftWallMuralMat>
}
```

그리고 `Architecture` 그룹 안(또는 `Looks` 하위 스코프를 새로 만들어서)에
머티리얼 정의 추가:

```usda
def Scope "Looks"
{
    def Material "LeftWallMuralMat"
    {
        token outputs:surface.connect = </World/Architecture/Looks/LeftWallMuralMat/PBRShader.outputs:surface>

        def Shader "PBRShader"
        {
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor.connect = </World/Architecture/Looks/LeftWallMuralMat/DiffTexture.outputs:rgb>
            float inputs:roughness = 0.9
            float inputs:metallic = 0.0
            token outputs:surface
        }

        def Shader "stReader"
        {
            uniform token info:id = "UsdPrimvarReader_float2"
            token inputs:varname = "st"
            float2 outputs:result
        }

        def Shader "DiffTexture"
        {
            uniform token info:id = "UsdUVTexture"
            asset inputs:file = @../textures/dining_extension_left.png@
            float2 inputs:st.connect = </World/Architecture/Looks/LeftWallMuralMat/stReader.outputs:result>
            token inputs:wrapS = "clamp"
            token inputs:wrapT = "clamp"
            color3f outputs:rgb
        }
    }
}
```

`RightWall_Mural` / `RightWallMuralMat`도 좌표(x=+5.98)와 텍스처 경로
(`dining_extension_right.png`)만 바꿔서 동일하게 추가.

## 4단계 — "2~3배 넓어 보이게" 만드는 핵심 포인트

- 사진 자체가 원근감(테이블이 멀어지며 작아지는)을 담고 있어야 "더 넓다"는
  착시가 생긴다. 벽에 딱 붙는 각도(정면)로만 찍으면 그냥 "벽지"처럼 보이고
  확장감이 안 남 — 반드시 비스듬한 원근 구도로 촬영할 것.
- Mesh의 `points` z 범위(`1.08 ~ 3.0`)는 `LeftWall_Upper`/`RightWall_Upper`
  밴드 높이(`1.08 ~ 2.8`)에 맞춘 것 — 실제 촬영 후 사진 비율에 따라 z 상단
  값(3.0)이나 y 범위(-4.6~4.6)를 조정해서 사진이 눌리거나 늘어나지 않게 할 것.

## 검증

Isaac Sim에서 씬을 열고 좌/우 벽을 봤을 때, 사진 속 테이블 줄이 실제 마지막
테이블 줄과 자연스럽게 이어져 보이면 성공. 이음매(사진 경계)가 선명하게
보이면 2단계(페더/블러) 보정을 다시 할 것.
