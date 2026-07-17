# Principled BSDF subset — 設計文書

umbreon の第 2 シェーディングモデル。`Material::model = ShadingModel::Principled` で
材ごとに opt-in し、既定(`Pov`)は従来の POV finish モデルを bit-exact に維持する。
UI(CueMol)は「金属的/プラスチック的/陶器的」等のプリセットを本 subset へ写像する
想定で、エンドユーザーに principled パラメータを直接触らせない方針
(2026-07 ユーザー確認)。対になる文書: [pt2_survey.md](pt2_survey.md)(輸送側)、
[api/libumbreon.md](api/libumbreon.md) §4.4.1(API リファレンス)。

## 1. パラメータと意味論

| パラメータ | 意味 |
|---|---|
| baseColor | 既存 pigment(頂点色 / プリミティブ色)を流用。新フィールドなし |
| `pbr.metallic` | dielectric(0) ↔ metal(1) の連続補間。metal は diffuse ローブ 0、F0 = baseColor |
| `pbr.roughness` | 知覚粗さ。**GGX α = roughness²**(Disney/glTF)。direct ハイライトと traced 反射を同じ幅で駆動(POV モデルで別々だった 2 つを 1 本化) |
| `pbr.specular` | dielectric F0 スケール: **F0 = 0.08 × specular**(0.5 → 0.04 = IOR 1.5 相当)。transmission を持たないため IOR は独立パラメータにしない |
| `pbr.anisotropy` / `anisotropyRotation` | Disney aspect 写像 aspect = √(1−0.9·anisotropy)、αx = α/aspect、αy = α·aspect。rotation は turn 単位。**sphere/cylinder 限定**(§4) |
| emission | `Material::emission` を共用。emissive NEE(mesh 三角形 + CSG 球/capped 円柱)は model 非依存で機能 |
| `Material::diffuse` | principled の **base weight** を兼ねる: diffuse albedo = diffuse×(1−metallic)×baseColor。POV からの変換で GI エネルギーが連続になる(giReflectance = kd×C の形が不変)。物理 albedo にしたければ 1.0 を設定 |

F0 の合成: `F0 = mix(Vec3(0.08 × pbr.specular), baseColor, pbr.metallic)`、Fresnel は
Schlick `F = F0 + (1−F0)(1−u)⁵`。

**Principled が無視する POV フィールド**: specular / roughness / brilliance / phong /
phongSize / metallic(bool)/ reflection、および `RenderOptions::specularScale`。

## 2. subset から落としたパラメータ(理由付き)

| パラメータ | 理由 |
|---|---|
| transmission(+屈折 IOR) | pt2_survey §1 の制約で対象外(ガラス/caustic ニーズなし)。既存 transparency(直進減衰 + group alpha)が分子用途の正解 |
| subsurface 系 | random-walk SSS はボリューム機構ごと新設の最重量級。分子表面に需要なし、「柔らかさ」は GI が担う |
| sheen | 布・粉塵用。縁の強調は subset の Fresnel と NPR エッジで足りる |
| coat / clearcoat | 存在意義は 2 ローブ(車塗装)。陶器・wet-look は滑らか dielectric 1 ローブで表現可。必要になったときの再追加第一候補 |
| specular tint | dielectric の色付きハイライトは物理的に疑わしく、金属色は metallic 経由で出る |
| thin film | 用途なし |
| 独立 IOR | transmission なしでは F0 の決定のみ = `pbr.specular` と冗長 |
| 独立 emission color | `emission × pigment` の既存規約(NEE の Le 定義含む)で足りる。後方互換で追加可能 |

## 3. エネルギー規約(direct pass)

POV 光単位(diffuse に 1/π なし: `diffuse 項 = kd·C·(N·L)·Lc`)を維持する。物理等価では
`E_irr = π·Lc` に相当するため、principled の specular direct 項は
**`π · f_ggx · (N·L) · Lc = π·D·G2·F/(4·cosθv) · Lc`**(π が単位系の補正)。

- diffuse = `diffuse×(1−metallic) · C · (N·L) · Lc`(Lambert、brilliance なし)。
  metallic=0 のとき POV の brilliance=1 diffuse と**式・結合順が一致 = bitwise parity**
- area light(`DistantLight::angularRadius`): 可視率×BRDF を**サンプルごとに同時評価**
  (POV の「平均可視率 × 中心方向 BRDF」と異なる非分離推定量)。ハイライトが影と同じ
  ソフトネスで広がる = pt2 phase 1 の「direct/間接の光源モデル一貫」のハイライト版
- fill light(highlight=false)は diffuse のみ(POV と同規則)。AO はハイライトを遮蔽しない
  (POV と同規則)
- **(1−F) の diffuse 補償は v1 ではしない**: dielectric F0 ≤ 0.08 で過大計上は ≲4%
  (grazing 除く)、metal は diffuse 0。目視で問題が出たら albedo スケール補償を再訪

## 4. anisotropy のフレーム規約

すべて hit の純関数(決定論・temporal 安定)。

| 形状 | フレーム | フォールバック |
|---|---|---|
| Sphere | pole = **world +Y(縦軸)**。t_φ = normalize(y×N)、t_θ = N×t_φ。CueMol 出力はカメラ座標系のため、z pole だと特異点が常にカメラ正対になる — 縦軸なら極は球の上下端(grazing)に隠れる | \|N·y\| ≥ 0.999 → 等方(hairy-ball の極特異点は位相的に不可避、固定軸で位置安定) |
| Cylinder 側面 | 軸 A の接平面射影 t = normalize(A − N·(A·N)) | cap 判定 \|dot(n̂g, A)\| ≥ 0.99 → 等方; 射影長² < 1e-12 → 等方 |
| Mesh | (v1 では常に等方) | per-vertex tangent API の導入後に解禁(NURBS/リボン刷新と同タイミング想定) |

軸は BuiltScene の per-segment side table(`cylAxis` / `cylCapAxis`)。sphere は N と
固定の縦軸(world +Y)のみで出るため保存不要。rotation は (t1,t2) を回転
(0.25 turn = 軸交換)。

## 5. integrator 方針(モード共通性)

- **direct shading は全モード共通**(basic raytracing / AO / cache / pt1 / pt2 — 唯一の
  local-shading 入口 shadeLocal の分岐)
- **diffuse 間接も共通**: kd の seam(giReflectance / pt1 gather / OIDN guide / cache
  oneBounceRadiance)が model 対応
- **specular 間接だけ段階劣化**: pt2 = traced mirror/glossy(E_spec、Fresnel per-sample)。
  それ以外 = フェイク環境項。額の規則(raytracing モード忠実度のための設計):
  `Material::reflection > 0`(変換材が持ち越した POV スカラー)なら **`reflection × bg`
  = POV のフェイク項と同額**; さもなければ **`F0 × bg`(定数、Schlick カーブなし)**。
  定数にする理由: Schlick の grazing 立ち上がりは明背景で全 dielectric の縁に白い
  シーンを乗せ、変換シーンの raytracing モード忠実度を壊す(POV の reflection 0 材には
  存在しない項)。Fresnel カーブは実ジオメトリに対して per-sample 評価される traced
  pt2 経路の担当。f0max == 0 かつ reflection == 0 で完全 skip(diffuse-only parity の
  必要条件)。pt2 では traced が所有するピクセルで skip され二重計上しない
- POV↔principled の混在シーンは材ごとに正しく共存する(reflF0 の中立既定 (1,1,1) で
  POV ピクセルの合成は bit 不変)

## 6. POV → principled 変換(bench、S4)

`--material <pov|principled>`(既定 pov)。scene_setup の post-parse 一括変換
(fromEdgeMacro 装飾は POV のまま)。写像は「同等の見た目クラス」への lossy 写像:
**bitwise 一致が保証されるのは diffuse-only 材(specular=0, phong=0, reflection=0,
brilliance=1)のみ**。POV↔principled は数学的に別ローブ(非正規化 Blinn + 2 ローブ加算
+ Fresnel なし ↔ 正規化 GGX 単ローブ + Schlick)なので、ハイライト・反射材の完全一致は
原理的に不可能(見た目が変わることが導入目的)。既定の principled 反転(S4b)は
目視承認後の専用コミットで行い、refactor_check baseline を意図的に再生成する。

**トゥーン/NPR finish は変換しない(ハイブリッド変換、2026-07-17 ユーザー指摘)**:
scene4 系のトゥーン 2〜3 値化は POV モデルの**非物理項そのもの**に依存する —
`phong 10000` はチャネルを飽和させてハードエッジの平坦ハイライトを作る
(エネルギー保存 BSDF のローブは F ≤ 1 で飽和し得ない)し、`brilliance 0` は
N·L 非依存の平坦 diffuse(Lambert は平坦になれない)。これは subset を
拡張しても解決しない(Blender も toon は Principled でなく専用 Toon BSDF)。
よって変換は `Material::toonLike()`(specular > 1 ‖ phong > 1 ‖ brilliance == 0
‖ unlit: diffuse・ハイライト・reflection すべて 0 の「nolighting」)を
**ShadingModel::Pov のまま残す**。混在シーンは S2 の中立性設計
(reflF0 = (1,1,1))が bit 保存を保証するテスト済み構成。

**toonLike は GI からも除外される(2026-07-17)**: toon のルックは
self-contained なので、(a) Route A の ambient→間接光置換を受けない
(nolighting の唯一の光源が ambient 項 — 置換すると kd=0 で真っ黒になる)、
(b) GI 下では ambient 項を**単位 ambient**で評価する(GI 時の
`Scene::ambientColor` は gather のエネルギー分配(bench では
`_light_inten×_amb_frac`)を運ぶ入れ物であって、toon の基準色ではない)。
これで toon 材は gi-off / pt1 / pt2 / 変換前後で ambient ベースが一致する。
基本/AO モードではシーン ambient と AO 変調に従来通り従う(AO の
bent-normal 勾配は ambient-only 材で検証されている慣行を壊さない)。
**残存する既知差**: bench は GI 時に直接光を減光する(既定 1.3→0.8、
エネルギーを ambient/GI 側へ移す再配分)ため、diffuse > 0 の toon 材の
直接光成分は GI レンダーでやや暗い。これは bench の照明モデル(.pov 経路)
の性質で API 経路では発生しない。固定したい場合は
`--declare _light_inten=1.3 --declare _amb_frac=0 ...` で非 GI エネルギーに
ピン留めできる(物理材の GI エネルギーも変わる点に注意)。

**`ShadingModel::Toon`(2026-07-17 実装)**: API 呼び出し側が NPR 意図を
**明示的に宣言**するためのタグ。Pov と同じ非物理ローブで shading され
(全ディスパッチが `model == Principled ? principled : pov` の二分岐なので
Toon は自動的に POV 側へ落ちる)、**ピクセルは 1 つも動かない**
(refactor_check 10/10 bit-exact)。効くのは `toonLike()` が値ヒューリスティックを
介さず常に true を返す点 — フィールド値が物理的に見える材でも GI 免除
(Route A)と principled 変換除外が確実に効く。`.pov` シーンは NPR 意図を
綴る手段がないため、Pov 側の値ヒューリスティックはそのまま残す
(bench の POV 再現性のため)。よって `toPrincipledMaterial` は toon を
**`ShadingModel::Pov` のまま**返す(retag しない): `--material principled` の
契約は「toon は不変で返す」であり、bench は再現性経路だから。

CueMol の「トゥーン」プリセット(toon1/toon2/nolighting/shadow)は
`ShadingModel::Toon` + Pov 相当の極端値フィールドへ写像する。より作り込んだ
NPR が欲しくなったら Blender Toon BSDF 型のパラメータ(bands/size/smooth)を
`Toon` に足す拡張余地を残す(未着手 — 現状の `Toon` はラベルであって
シェーダではない)。

写像表(`src/bench/material_convert.cpp`、S4 実装済み):

| POV finish | principled |
|---|---|
| ambient / diffuse / emission | 据置(共有意味論; diffuse = base weight) |
| `metallic`(bool、F_Metal* プリセット含む) | `pbr.metallic = 1` |
| `specular s` + `roughness r` | e = 1/r(blinnExp)、α = √(2/(e+2))、`pbr.roughness = √α`、`pbr.specular = clamp01(s)` |
| `phong p` + `phong_size ps`(specular なし時) | e = max(1, ps) で同上、`pbr.specular = clamp01(p)` |
| ハイライトなし | `pbr.specular = 0`(dielectric ローブなし) |
| `reflection > 0`、ハイライトなし | `pbr.metallic = 1, pbr.roughness = 0`(研磨金属ミラー、F0 = pigment) |
| `reflection > 0` + ハイライトあり | **`pbr.metallic = max(既値, clamp01(reflection))`** — dielectric F0 上限 0.08 では reflection 0.3 の床がほぼ消えるための部分金属化。**要目視判定のヒューリスティック**(代替 = metallic 0 の物理プラスチック) |
| brilliance / metallic tint | 写像後は読まれない(破棄を文書化) |
| `reflection` スカラー | **持ち越し**: Principled の BSDF は読まないが、非 pt2 モードのフェイク環境項が POV と同額の `reflection × bg` として使う(raytracing モード忠実度。§5 参照) |

## 7. E_spec denoise の解像度(S5、実測 2026-07-17)

計測(1ab0_scene4_materials × `--material principled`、700² 出力、pt2 既定):

| 構成 | 全体時間 |
|---|---|
| ss=3、E_spec OIDN を render grid(2100²)で実行(旧) | 6.13s(OIDN 分 +3.7s = 全体の 60%) |
| ss=3、`--pt2-glossy-denoise off` | 2.41s |
| **ss=3、out-res 往復(採用)** | **2.77s(OIDN 分 +0.40s、9.3 倍短縮)** |
| ss=1(経路不変) | 1.07s |

採用した修正: supersample > 1 のとき、E_spec とそのガイドを **glossy マスク付き
box-downsample で出力解像度へ落として OIDN を実行**し、`upsampleJointBilateral`
(diffuse E と同じ normal/depth edge-stop)で render grid に復元して composite する。
ss=1 は従来経路のまま bitwise 不変。ss² の box-average が捨てるはずのディテールの
ために ss² 倍の OIDN を払わない、という整理。決定論は per-cell の固定順序 reduction
+ 既存 upsampler で維持(スレッド数不変をテストで固定)。
