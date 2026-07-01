# pt1 Phase 0 調査報告: リポジトリ構造と pt1 統合ポイント

`docs/plans/PLAN_pt1_integrator.md` Phase 0-1 の調査 6 項目のまとめ。指示書の想定との差異と、
pt1 の適合方針は各項目の末尾に記す。

## 1. メッシュの内部表現

- **型**: `struct Mesh`(`src/umbreon/scene.hpp`)
  - `std::vector<Vec3> positions` / `normals`(頂点法線。空なら無し → 幾何法線フォールバック)
  - `std::vector<Vec4> colors`(頂点ごとの pigment RGB + opacity)
  - `std::vector<uint32_t> index`(空なら de-indexed の triangle soup)
  - `Material material` + `std::vector<Material> materials` / `triMaterialId`(三角形ごとの上書き。
    `materialForTri(primID)` でアクセス)
- **albedo の持ち方**: 単一の albedo 配列は無い。実効 diffuse 反射率は
  `material.diffuse (kd) × 頂点補間 pigment (colors)`。GI パスはこれを per-pixel の
  `giReflectance` として一次ヒット時に取得する(`hit_shader.hpp`)。
- Embree には `RTC_GEOMETRY_TYPE_TRIANGLE` として登録済み(`render/scene_build.cpp`)。頂点属性
  slot 0 = 法線、slot 1 = 色で `rtcInterpolate0` 補間可能。
- **CSG プリミティブ**(原子=球、結合=円柱)も Embree シーンに実ジオメトリとして存在するが、
  GI は `GeomKind::Mesh` のみ対象(既存 cache と同じ扱い。gather レイが当たった場合は黒の
  オクルーダ扱い — `oneBounceRadiance` が非メッシュで 0 を返す)。

## 2. カメラモデルと primary ray 生成

- **型**: `struct Camera`(`scene.hpp`): `position` / `direction` / `up` / `fovy`(度)/
  `orthographic` / `height`。
- **基底構築**: `embree_renderer.cpp` の `render()` 冒頭(~234-248 行)で
  `dir/right/trueUp` の正規直交基底と、perspective の `persHalfW/persHalfH = tan(fovy/2)·aspect`
  または orthographic の `halfW/halfH` を計算。
- **primary ray**: メインのピクセルループ(同 ~350-445 行)でピクセル中心に 1 本。
  スーパーサンプリングは「hi-res W×H で描画 → box downsample」方式(`pipeline.cpp`)。
- pt1 の半解像度パスは同じ基底式で独立に primary ray を撃つ(`tracePt1GBuffer`、Phase 5)。

## 3. 光源表現と直接照明

- **光源は DistantLight のみ**(`scene.hpp`): `direction`(光の進行方向、POV 規約)、`color`、
  `intensity`、`castsHighlight`。点光源・面光源・環境マップは存在しない。
  - 指示書 Phase 2 の「面光源/点光源の NEE」は該当なし。シャドウレイの
    `tfar = 光源距離 − 2eps` の注意点も distant light では `tfar = ∞` となり該当しない。
- 環境ドームライト(`--env-lights`、`ao/env_dome.hpp`)は distant light を合成生成する
  **直接光**。GI の gather sky と併用すると二重計上になるため GI 時は使用しない(pt1 は
  警告を出す)。
- 定数 ambient: `scene.ambientColor × ambientIntensity`。**GI 有効時はメッシュヒットで
  ゼロ化**され(`hit_shader.hpp` の gi ゲート)、GI の gather がそのエネルギーを
  occlusion-aware に回収する。ベンチ側は `_amb_frac` で直接光→ambient のエネルギー再配分を
  行う(`bench/main.cpp`)。
- **直接照明評価**: `shading/secondary_rays.hpp` の `computeShadow()`(`rtcOccluded1` ベース、
  ソフトシャドウ対応)+ N·L。gather ヒット点での再評価は
  `experimental/irradiance_cache/irradiance_cache.hpp` の `oneBounceRadiance()` が
  そのまま使える(シャドウテスト済み Σ N·L·color × kd·pigment を返す。emission 加算なし)。

## 4. 既存 irradiance cache インテグレータの呼び出し口

- **分岐点**: `EmbreeRenderer::render()`(`render/embree_renderer.cpp:452`)の
  `if (opt.gi && meshPresent(built))` ブロック。メインループ(直接照明)完了後の
  ポストパスとして、live BVH 上で cache 構築 → per-pixel 補間 → 合成
  `res.color += giIntensity × giReflectance × E` + `res.indirect` AOV 書き込み(~532-541 行)。
- **pt1 の差し込み方**: このブロック内を `opt.giIntegrator`(0=cache / 1=pt1)で分岐する。
  直接照明・シェーディングは共通(指示書 Phase 2 の「直接照明一致」が構成上保証される)。
- **エネルギー規約**(重要): cache は `E_stored = mean(L_i) = E_true/π` を保存し(1/π 係数
  なし、`irradiance_cache.hpp` の gradient コメントに明記)、合成側も 1/π を掛けない。
  pt1 も同一規約を採用する。指示書 3-1 の推定量 `(π/N)ΣL_i` は本 repo では
  `E_stored = (1/N)ΣL_i` に対応する。

## 5. 画像バッファ形式と出力

- **`FrameResult`**(`render/render_types.hpp`): linear HDR の平面 float 配列。
  `color`(RGBA)、`albedo`、`normal`、`depth`(レイ距離)、`position`、`indirect`、
  `giOcclusion`、`giRecordViz` ほか。
- **出力**: PNG / PPM のみ(`bench/image/image_io.cpp`)。**EXR writer は存在しない** →
  pt1 の AOV 確認は PNG(既存 `--dump-aov`)で行う。
- **色空間**: 内部はすべて linear。出力直前に `assumed_gamma` 適用(`pipeline.cpp`)→
  PNG 書き込み時に sRGB エンコード。PPM は linear のまま。
- スーパーサンプリング時、GI ポストパスは hi-res グリッドで走り、`indirect`/`position` 等も
  `pipeline.cpp` でダウンサンプルされる。pt1 の「半解像度」はこのレンダーグリッドの半分を
  指す(ベンチ比較は `--supersample 1` で行う)。

## 6. 並列化とビルドシステム

- **並列化**: oneTBB。cache fill は `tbb::parallel_for` + `blocked_range`、per-pixel 補間は
  行単位。乱数は `tea2` ハッシュ(`secondary_rays.hpp`)をインデックスから決定的に生成し、
  スレッド間共有なし(スレッド数非依存)。pt1 は `blocked_range2d` の 16×16 タイルで
  同じ流儀に従う。指示書の PCG32 は導入せず、意図(決定的 seed・非共有)を `tea2` で満たす。
- **自己交差対策**: `selfIntersectEps()`(`secondary_rays.hpp`、OSPRay 式のスケール適応
  epsilon)が全セカンダリレイで使用済み。指示書の `1e-4 × sceneScale` 固定値より頑健なので
  こちらを再利用する。
- **ビルド**: CMake(3.20+)+ go-task(`Taskfile.yml`)。`task build` / `task test`(CTest)。
- **依存ライブラリはすべて導入済み**(指示書 Phase 0-2 は作業不要):
  - Embree 4.4.1: `find_package(embree 4 CONFIG)`
  - oneTBB: `find_package(TBB CONFIG)`
  - OIDN 2.5: `find_package(OpenImageDenoise 2 CONFIG)`、`-DUMBREON_WITH_OIDN=ON`
    (deplibs バンドルに同梱、`task build` が自動で有効化)
  - 入手経路: `task deps:fetch`(CueMol deplibs のプリビルド静的ライブラリ、
    Linux/macOS/Windows x64 + macOS arm64)、または `task deps`(brew / apt のシステム
    パッケージ)、または `task deps:build`(ソースビルド・フォールバック)。
- **OIDN 統合済みコード**: `experimental/irradiance_cache/denoise_oidn.cpp`(`hdr=true` 設定済み、
  albedo/normal ガイド対応、任意解像度対応)。ただし NaN 除去は無いので pt1 側で scrub する。

## 指示書との主な差異まとめ

| 指示書の想定 | repo の実態 | pt1 の適合 |
|---|---|---|
| 依存 3 ライブラリを新規導入 | すべて導入・使用済み | 作業不要(本書に手順のみ記載) |
| 面光源/点光源の NEE | distant light のみ | `computeShadow` + `oneBounceRadiance` 再利用 |
| 独立した直接照明パス | メインループが直接照明を担当 | 無変更で共用(一致が自動成立) |
| PCG32 | `tea2` ハッシュ | `tea2` を per-pixel seed で再利用 |
| `eps = 1e-4·sceneScale` | `selfIntersectEps()`(適応) | 既存を再利用 |
| EXR 出力 | PNG/PPM のみ | PNG + `--dump-aov` |
| 半解像度 = 出力の W/2×H/2 | GI は supersample 後グリッドで走る | レンダーグリッドの半分。ベンチは `--supersample 1` |
