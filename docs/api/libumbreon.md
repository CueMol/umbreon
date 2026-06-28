# libumbreon API ガイド（CueMol2/3 統合の起点）

`libumbreon` は CueMol の production rendering バックエンドである **POV-Ray の代替**として
作られた、Embree 4 ベースのオフラインレンダラです。CueMol2/3 から **静的リンクして
組み込む**ことを想定しています。このドキュメントは、その統合実装のスタートポイントです。

> 関連: 実装プラン `docs/plans/ao-soft-shadow.md`、最小サンプル `examples/minimal_render.cpp`。

---

## 1. 概要 — 何をするライブラリか

- **入力**: コード上で `umbreon::Scene`（ジオメトリ・カメラ・ライト・マテリアル・フォグ）を組み立てる。
- **呼び出し**: `umbreon::render(scene, options)` を1回呼ぶ。
- **出力**: `umbreon::FrameResult`（linear HDR の RGBA フレームバッファ＋深度）。
- POV-Ray の SDL パーサも OSPRay も**不要**。依存は **Intel Embree 4 + oneTBB のみ**。
- CueMol は **POV ファイルを経由せず**、自前のシーン表現から直接 `umbreon::Scene` を構築する
  （`.pov`/`.inc` パーサは `umbreon_cli`／bench 専用で、ライブラリには含まれない）。

### レンダリングモデル

primary ray + **POV 忠実なローカルシェーディング**（per-geometry の POV finish）。single-bounce。

| 機能 | 対応 | 制御 |
|---|---|---|
| 直接光（distant light）+ 環境光 | あり | `Scene::lights`, `Scene::ambientColor` |
| Ambient Occlusion（メッシュのみ） | あり（既定 OFF） | `RenderOptions::aoSamples` ほか |
| 影（ハード／ソフト＝エリアライト） | あり（既定 OFF） | `RenderOptions::shadows` ほか |
| 透過（front-to-back over ＋ group veil） | あり | `RenderOptions::transparency`, `Scene::veilGroups` |
| OpenGL 線形 fog（深度ベース post-process／透過時 alpha フェード） | あり | `Scene::fog` |
| Phong / Blinn / metallic ハイライト | あり | `Material` |
| 安価な反射（`reflection * background`） | あり | `Material::reflection` |
| 大域照明（GI / radiosity）、屈折、二次反射 | **なし**（設計上スコープ外） | — |

---

## 2. ビルドとリンク

### 依存

- C++17 コンパイラ
- **Intel Embree 4** + **oneTBB**（静的リンク）

取得方法:
- **Linux x64 / macOS x64・arm64 / Windows x64（推奨）**: `task build:static` が CueMol2 の
  **deplibs**（CueMol 本体と同一のプリビルド静的 Embree 4 + TBB）を取得して静的リンクする。
  CueMol との依存バージョン一致が保証される。
- **その他のプラットフォーム**: `brew install embree tbb`、または `task deps:build`（ソースから静的ビルド）。
  詳細はリポジトリ README の「Build」を参照。

`libumbreon` は **静的ライブラリ**なので、Embree/TBB のシンボルは最終リンク時に解決されます。
つまり **consumer（CueMol）側でも Embree 4 と TBB をリンクする必要があります**
（`umbreon::umbreon` ターゲットがこれらを PUBLIC 依存として伝播します）。

公開ヘッダ（`<umbreon/...>`）は **Embree/TBB のヘッダを一切 include しません**。
API を使うのに Embree のヘッダは不要で、必要なのはリンクだけです。

### consumer からの利用方法（2通り）

**(A) find_package（インストール済みパッケージを参照）** — 推奨

```sh
# 1) umbreon をビルドしてインストール
cmake -S . -B build
cmake --build build
cmake --install build --prefix /opt/umbreon     # 任意の prefix

# 2) CueMol 側 CMake
#    CMAKE_PREFIX_PATH に umbreon の prefix と Embree/TBB(Homebrew) を含める
find_package(umbreon REQUIRED)
target_link_libraries(cuemol_render PRIVATE umbreon::umbreon)
```

`find_package(umbreon)` は内部で `find_dependency(embree 4)` と `find_dependency(TBB)` を
呼ぶため、`CMAKE_PREFIX_PATH` に Embree/TBB（macOS では `/opt/homebrew`）も含めてください。

**(B) add_subdirectory（ソースを同梱）**

```cmake
add_subdirectory(third_party/umbreon)
target_link_libraries(cuemol_render PRIVATE umbreon::umbreon)
```

どちらの場合もリンクするターゲット名は **`umbreon::umbreon`** です。

### 公開ヘッダ

統合に必要なのは **1つ**です:

```cpp
#include <umbreon/umbreon.hpp>   // render() + Scene/RenderOptions/FrameResult を取り込む
```

これが推移的に `<umbreon/scene.hpp>`（ジオメトリ／カメラ／ライト／マテリアル型）と
`<umbreon/render/render_types.hpp>`（`RenderOptions`／`FrameResult`）を取り込みます。

画像後処理ヘルパ（`srgbEncode8` / `applyAssumedGamma` / `boxDownsample`）は別ヘッダ
`<umbreon/postprocess/image_ops.hpp>` に分離されています。エッジ関連の公開ヘッダは
`<umbreon/edges/{stroke_edges,mesh_feature_edges,object_space_edges}.hpp>` です。

> **移行メモ（フォルダ再編）**: 以前 `<umbreon/render/>` 配下にあった公開ヘッダのうち、
> `srgbEncode8` 等は `umbreon.hpp` から `<umbreon/postprocess/image_ops.hpp>` へ、
> エッジ系3ヘッダは `<umbreon/render/...>` から `<umbreon/edges/...>` へ移動しました。
> `render_types.hpp` は `<umbreon/render/render_types.hpp>` のまま据え置きです。

バージョンは `UMBREON_VERSION_MAJOR` / `_MINOR` / `_PATCH` マクロで参照できます。

---

## 3. クイックスタート（最小例）

完全に動くサンプルは `examples/minimal_render.cpp`（`examples/CMakeLists.txt` で
`find_package(umbreon)` してビルド）。要点のみ抜粋:

```cpp
#include <umbreon/umbreon.hpp>

umbreon::Scene scene;

// 1) ジオメトリ: 三角形メッシュ。`index` を省略すると corner==vertex の自明
//    マッピング（= de-indexed: 三角形 i は頂点 [3i, 3i+1, 3i+2]）になる。§4.3
umbreon::Mesh& mesh = scene.mesh;
for (const umbreon::Vec3& c : corners) {            // 6 頂点 = 2 三角形 (quad)
  mesh.positions.push_back(c);
  mesh.normals.push_back({0, 0, 1});                // 頂点法線
  mesh.colors.push_back({0.2f, 0.4f, 0.8f, 1.0f});  // rgb + opacity(=alpha)
}                                                   // mesh.index は空のまま
mesh.material.ambient = 0.2f;                        // POV finish
mesh.material.diffuse = 0.8f;

// 2) カメラ（orthographic, -Z を向く）
scene.camera.position = {0, 0, 5};
scene.camera.direction = {0, 0, -1};
scene.camera.up = {0, 1, 0};
scene.camera.orthographic = true;
scene.camera.height = 2.5f;

// 3) ライト（distant）
umbreon::DistantLight light;
light.direction = {0, 0, -1};   // 光が進む向き（→ +Z 面を照らす）
scene.lights.push_back(light);

// 4) オプション
umbreon::RenderOptions opt;
opt.width = 256;  opt.height = 256;
opt.supersample = 2;        // アンチエイリアス
opt.aoSamples = 16;         // AO（0 = OFF）
opt.shadows = true;         // 影

// 5) レンダリング
umbreon::FrameResult frame = umbreon::render(scene, opt);

// 6) linear HDR float RGBA -> 8-bit sRGB バイト列（CueMol 側の画像パイプラインへ）
//    srgbEncode8 は <umbreon/postprocess/image_ops.hpp> で提供される。
std::vector<std::uint8_t> rgb = umbreon::srgbEncode8(frame, 3);
```

---

## 4. API リファレンス

### 4.1 エントリポイント

```cpp
FrameResult render(const Scene& scene, const RenderOptions& opt);
```

パイプライン（内部順序）:
`supersample（opt.width*ss × opt.height*ss で描画）` → `Embree primary-ray シェーディング
(+AO +影)` → `OpenGL 線形 fog（scene.fog, 平面 eye-z）` → `linear box-downsample` → `assumed_gamma（scene.assumedGamma）`。

- `opt.width` / `opt.height` は **最終出力**サイズ（supersample 前ではない）。
- 戻り値は **linear HDR** のフレームバッファ（後段で sRGB エンコードする）。
- **スレッド**: 内部で TBB により行並列（既定で全コア）。スレッド数を絞りたい場合は
  呼び出し側で `tbb::global_control` を使う。
- **例外**: Embree のデバイス生成／シーン構築に失敗すると `std::runtime_error` を投げる。
- **状態**: `render()` は呼び出しごとに Embree デバイスを生成・破棄する（ステートレス）。
  そのため**同一プロセスから順に**何度でも呼べる。複数スレッドからの**同時**呼び出しは
  非対応（内部で TBB を使うため）。連続フレームは逐次呼び出すこと。
  （注: 1フレームあたりデバイス＋BVH を再構築するため、対話的に多数フレームを回す用途では
  将来 `Renderer` ハンドルの導入を検討。§7 参照。）

### 4.2 `Scene` — 構築する対象

`render()` が参照するフィールド:

| フィールド | 型 | 意味 |
|---|---|---|
| `mesh` | `Mesh` | 三角形メッシュ（surface / density map surface 等）。§4.3 |
| `spheres` | `vector<Sphere>` | CueMol の ball / シルエット接合点。§4.3 |
| `cylinders` | `vector<Cylinder>` | stick / シルエットエッジ。§4.3 |
| `instanceOffsets` | `vector<Vec3>` | 全ジオメトリを各オフセットへ複製（空＝単一コピー） |
| `camera` | `Camera` | §4.5 |
| `lights` | `vector<DistantLight>` | §4.5 |
| `ambientColor` | `Vec3` | 環境光ラジアンス（既定 `{1,1,1}`）。ambient 項 = `material.ambient * pigment * ambientColor` |
| `background` | `Vec3` | 背景色（linear） |
| `fog` | `Fog` | POV fog（§4.5）。`fog.enabled` で有効化 |
| `assumedGamma` | `float` | POV `assumed_gamma`。出力 RGB を `pow(c, gamma)`（既定 1.0 = 無変換） |
| `veilGroups` | `vector<uint16_t>` | 加算「veil」として扱う透過グループ id。§4.6 |

> 補足: `Scene::ambientIntensity` と `Scene::aoDistance` は `render()` からは**直接読まれない**
> （CLI/ビルダ用のキャリア）。CueMol からは環境光は `ambientColor`、AO 半径は
> `RenderOptions::aoDistance` で指定する。

### 4.3 ジオメトリ

**`Mesh`**（任意 index 付き三角形メッシュ。`index` 空＝de-indexed: 三角形 i は頂点 `[3i, 3i+1, 3i+2]`）

| フィールド | 型 | 意味 |
|---|---|---|
| `positions` | `vector<Vec3>` | **頂点ごと**の位置（サイズ = `vertexCount()`） |
| `normals` | `vector<Vec3>` | 頂点ごとの法線（補間してシェーディング法線に使用） |
| `colors` | `vector<Vec4>` | 頂点ごとの `rgb + opacity`（w = 不透明度, 1 = 不透明）。linear |
| `index` | `vector<uint32_t>` | 三角形コーナー → 頂点（サイズ = `3 × 三角形数`）。**空＝自明マッピング**（corner c → vertex c = 旧来の三角形スープ） |
| `posClass` / `posClassCount` | `vector<int32_t>` / `int32_t` | 頂点 → 位置クラス id（**位置のみ**で溶接した ~1e-4 トポロジ。エッジ抽出が使う）。空＝抽出器が自前で溶接 |
| `material` | `Material` | 単一マテリアル時に使用 |
| `materials` / `triMaterialId` | `vector<Material>` / `vector<uint8_t>` | 三角形ごとに異なる finish を使う場合（空＝`material`） |
| `triGroupId` | `vector<uint16_t>` | 三角形ごとの透過グループ（CueMol section）。空＝全て group 0 |

アクセサ: `vertexCount()` / `cornerCount()`（= `index.size()`、空なら `positions.size()`）/ `triangleCount()`
（= `cornerCount()/3`）/ `cornerVertex(corner)`（コーナー → 頂点スロット。`index` を尊重する唯一の継ぎ目）/
`toDeindexed()`（index/posClass を捨てた三角形スープ複製。値はコピーで bit-exact）。consumer は頂点配列を
`positions[3i+k]` と直接仮定せず **必ず `cornerVertex()` 経由**で参照する。

> **`Mesh` のトポロジーと溶接**: `Mesh` は **任意 index 付き**。`index` が空なら de-indexed（三角形スープ）
> = 三角形 i の3頂点を `positions[3i], [3i+1], [3i+2]` に並べる退化ケースで、手組みメッシュや `toDeindexed()`
> がこれ。`.pov` を読む通常パスでは **load 時に 1 回だけ溶接して indexed mesh を構築する**
> （`mesh2_reader.cpp::buildIndexedBlock`、頂点 ~6 倍削減）。溶接基準はレンダリングとトポロジで異なる:
> - **描画用 `index`** … `(位置, 法線, 色)` の**ビット一致**で頂点を併合。ハードエッジや色境界は別頂点のまま
>   残るので**座標を一切動かさず描画は bit-identical** を維持する。
> - **`posClass`（トポロジ用）** … **位置のみ**で溶接した位置クラス（既定 ~1e-4）。ハードエッジでも両面が
>   同一クラスを共有しないと crease が border に誤判定されるため、法線・色の分割を無視する別基準が要る。
>
> POV mesh2 の `face_indices` は接続性を完全には符号化しない（CueMol はリボンを巻きごとの triangle strip で
> 出力し、巻き境界の頂点・法線が重複するため、隣接巻きとの接続は座標一致としてしか存在しない）。
> エッジ抽出（`--edges` / `--obj-edges`）は `posClass` で隣接を見て water-tight トポロジーを判定する
> （`posClass` 不在なら抽出器が同じ位置溶接を自前で再現する fallback）。溶接の単一定義は
> `edges/mesh_weld.hpp`、詳細・限界は `edges/mesh_feature_edges.cpp` の step 1 を参照。

**`Sphere`**: `center`, `radius`, `color`(rgb+opacity), `material`（既定 `Material::flatOutline()`）, `group`。

**`Cylinder`**: `p0`, `p1`, `radius`, `color`, `material`, `group`,
`opacity1`（p1 側の不透明度。`edge_line2` の透過グラデーション用。`< 0` で `color.w` 一定）,
`open`（**重要**: `true` = キャップ無しのシルエットエッジ。共有端点で連結し継ぎ目の
ビーズを防ぐ。`false` = フラットキャップの bond/wireframe）。

> 球・シリンダは既定で `Material::flatOutline()`（ambient 1 / diffuse 0 = 生色のフラット表示）。
> **AO も影も outline プリミティブには適用されない**（シルエットが誤って暗くならない）。

### 4.4 `Material`（POV finish）

| フィールド | 既定 | 意味 |
|---|---|---|
| `ambient` | 0.2 | 環境光係数（AO はこの項のみを減光） |
| `diffuse` | 0.8 | 拡散係数 |
| `specular` | 0.0 | Blinn ハイライト量 |
| `roughness` | 0.02 | Blinn の粗さ（小さいほど鋭い） |
| `brilliance` | 1.0 | 拡散の N·L 指数 |
| `phong` | 0.0 | Phong ハイライト量 |
| `phongSize` | 40.0 | Phong 指数 |
| `metallic` | false | ハイライトを pigment 色で着色（POV metallic） |
| `reflection` | 0.0 | `reflection * background` を加算（安価な反射） |
| `emission` | 0.0 | 自己発光 |

`Material::flatOutline()` は ambient 1 / diffuse 0 / specular 0 を返し、`ambientColor = {1,1,1}` の
下で「pigment 色そのまま」のフラット表示になる（シルエット線用）。

### 4.5 カメラ・ライト・フォグ

**`Camera`**: `position`, `direction`（視線・正規化）, `up`, `orthographic`(bool),
`height`（ortho 時の像面高さ・world 単位）, `fovy`（perspective 時の垂直画角・度）。
カメラ基底は `right = cross(direction, up)`、`trueUp = cross(right, direction)`。
ピクセル原点は**左上**（行 0 = 像面上端）。

**`DistantLight`**: `direction`（**光が進む向き**。例: 真上から照らすなら `{0,-1,0}`）,
`color`, `intensity`, `castsHighlight`（false = POV の `shadowless`/fill light = 拡散のみ、
ハイライト無し）。

**`Fog`**: **OpenGL 線形 fog**（CueMol のインタラクティブ表示に一致）。`enabled`, `color`（= 背景色）,
`start` / `end`（平面 eye-z）。係数 `f = clamp((end - z)/(end - start), 0, 1)`（`f=1` で `start` 以近＝素色、
`f=0` で `end` 以遠＝fog 色）を平面 eye-z（`FrameResult::viewZ`）に対する post-process として適用する。
- **不透明背景**（`transparentBackground=false`）: RGB を `mix(color, objRGB, f)`、alpha 不変（GL 完全一致）。
- **透過背景**（`true`）: fog 色を焼き込まず**被覆を `alpha *= f` でフェード** → ストレート alpha 出力を
  任意背景へ "over" 合成でき、遠方は合成先の色へ正しく溶ける（`B=color` で不透明描画と厳密一致）。

POV リーダが CueMol の POV ground-fog ハック（`distance=slabDepth/3`）から `start=_distance`,
`end=start+1.5*distance` を復元して格納する。レガシー `distance`/`type`/`offset`/`alt`/`up` は互換のため
残置（`distance` のみ復元に使用、ground-fog 経路は廃止）。

### 4.6 `RenderOptions`

`render()` が honor する**全フィールド**（既定値は「全 secondary 効果 OFF」＝素の primary-ray 表示）:

| フィールド | 既定 | 意味 |
|---|---|---|
| `width` / `height` | 1024 / 768 | 最終出力サイズ（px） |
| `supersample` | 1 | `width*ss × height*ss` で描画し linear で box 平均（AA）。1 = OFF |
| `aoSamples` | 0 | メッシュヒットあたりの AO レイ数。**0 = AO OFF**（既定で bit-exact） |
| `aoDistance` | 1e20 | AO オクルーダ探索半径（ray tfar, world 単位）。シーン径の数分の1程度が目安 |
| `aoIntensity` | 1.0 | AO 強度: `aoFactor = 1 - aoIntensity*(1-rawAO)` |
| `aoFalloffPower` | 0.0 | 0 = binary（legacy）。> 0 で距離減衰 `(max(0,1-t/R))^power`（遠方遮蔽を緩和） |
| `aoMultiScale` | false | true で 3 段ネスト半径 `{0.08,0.30,1.0}*aoDistance`（接触影＋形状陰影を両立） |
| `aoBentNormal` | false | true で bent normal に沿った 2 色 sky/ground 方向性 ambient |
| `aoSkyColor` / `aoGroundColor` | `{1,1,1}` | bent normal 勾配の上/下半球の色（× `scene.ambientColor`） |
| `aoUseCameraUp` / `aoUp` | true / `{0,1,0}` | 勾配軸：カメラ up（view-stable）か明示世界軸 |
| `aoMultibounce` | false | true で albedo 別 GTAO cubic（淡色凹部の過暗化を抑制） |
| `aoLowDiscrepancy` | false | true で Hammersley + per-pixel Cranley-Patterson（同サンプル数で AO 分散↓） |
| `aoDiffuseFactor` | 0.0 | 0 = ambient のみ。> 0 で凹部の直接 diffuse も減光（粗い間接遮蔽近似） |
| `aoWriteAov` | false | true で AO/G-buffer AOV（albedo/normal/contact/shape/bent/avgHitDist）を `FrameResult` へ出力。色は不変 |
| `shadows` | false | ライトからの影を落とす。false = OFF |
| `shadowSamples` | 1 | ライトあたりの影レイ数（> 1 でソフト＝エリアライト） |
| `lightRadius` | 0.0 | ライトの角半径（度）。> 0 でソフト影（penumbra） |
| `specularScale` | 1.0 | 各マテリアルの specular 量に乗算 |
| `transparency` | true | front-to-back 透過 walk。false = 不透明のみ（最前面で停止） |
| `transparentBackground` | false | 背景の被覆 0 → 出力 alpha = 累積被覆（POV `_transpbg`）。fog 有効時は fog 色を焼かず `alpha *= f` でフェード（§4.5） |
| `maxTransparentLayers` | 256 | 1レイあたり透過ヒット数の安全上限（通常は alpha 早期終了で停止） |

> AO/影は既定 OFF なので、`RenderOptions` を既定構築すると現行の POV マッチ出力（bit-exact）に
> なる。AO/影を使う場合は該当フィールドを設定する。
>
> 立体感プリセット指針: 滑らかな density surface では「広域の窪み暗化」が立体感の主因なので
> `aoMultibounce=true` + `aoDiffuseFactor≈0.6`（大半径 AO を残し凹部を深く落とす）が有効。
> `aoFalloffPower`/`aoMultiScale` は遠方遮蔽を割り引くため、滑らかな面ではむしろ平坦化する点に注意
> （接触影・方向性が要るモデルでは有効）。詳細は `docs/plans/ao-quality.md` の「プリセット指針」。

### 4.7 `FrameResult`

| フィールド | 型 | 意味 |
|---|---|---|
| `width` / `height` | `int` | 出力サイズ |
| `color` | `vector<float>` | `width*height*4` の **linear HDR RGBA**、**左上原点** |
| `depth` | `vector<float>` | `width*height`、カメラからのレイ距離（背景は 0） |
| `viewZ` | `vector<float>` | 平面 eye-z（背景は 0）。**stroke edges 有効時 or fog 有効時のみ充填**（線形 fog が参照）。それ以外は空 |
| `albedo` / `normal` | `vector<float>` | first-hit AOV（OIDN guide / cache 空間キー）。**`aoWriteAov` 有効時に主経路で充填**（normal は stroke edges 有効時も）。それ以外は空 |
| `contactAo` / `shapeAo` | `vector<float>` | AO の接触/形状成分（未ブレンド）。**`aoWriteAov` 有効時のみ充填** |
| `bentNormal` | `vector<float>` | `width*height*3` 平均非遮蔽方向。**`aoWriteAov` 有効時のみ充填** |
| `avgHitDist` | `vector<float>` | 平均オクルーダ距離（world）。暗さ要因の切り分け用。**`aoWriteAov` 有効時のみ充填** |
| `renderSeconds` | `double` | レンダ時間 |
| `effectiveTriangles` | `size_t` | 実効三角形数（instance 込み） |

### 4.8 ポストプロセスユーティリティ

ヘッダ: `<umbreon/postprocess/image_ops.hpp>`（`namespace umbreon`）。

```cpp
// linear RGBA -> 8-bit sRGB バイト列。channels は 3(RGB) か 4(RGBA; alpha は linear 格納)。
std::vector<std::uint8_t> srgbEncode8(const FrameResult& frame, int channels);

// RGB を pow(c, g) する（in-place, POV assumed_gamma; g==1 は no-op, alpha 不変）。
void applyAssumedGamma(FrameResult& frame, float g);

// linear 画像を w×h から (w/ss)×(h/ss) へ box 平均。
std::vector<float> boxDownsample(const std::vector<float>& src, int w, int h,
                                 int channels, int ss);
```

> `render()` は linear HDR を返す。表示・保存用には `srgbEncode8()` で 8-bit sRGB に変換し、
> その**バイト列を CueMol 側の画像保存に渡す**（PNG/PPM 書き出しはライブラリには含まれない）。

---

## 5. 規約

- **ピクセル原点**: 左上（`color`/`depth` とも row 0 = 画像上端）。
- **色空間**: シーンの色・出力 `color` はすべて **linear**。最終表示時に sRGB エンコード。
- **gamma**: `Scene::assumedGamma` が出力 RGB に `pow(c, gamma)`（POV `assumed_gamma`）。
- **不透明度**: 色の w チャネル（1 = 不透明, 0 = 完全透過）。
- **ライト向き**: `DistantLight::direction` は「光が進む向き」（surface→light ではない）。
- **カメラ**: `right = cross(direction, up)`、`trueUp = cross(right, direction)`。
- **単位**: world 単位は入力ジオメトリのものをそのまま使う（umbreon は単位を仮定しない）。
  `aoDistance` / `lightRadius`（度）/ `camera.height` も同じ world 単位／度。

---

## 6. パフォーマンスとエラー処理

- **並列化**: TBB による行並列。`supersample` は描画コストを `ss²` 倍、`aoSamples` は
  メッシュヒットあたり `aoSamples` 本、`shadowSamples` はライト×影サンプル分の二次レイを足す。
  視覚用途の目安は AO ≥ 16、ソフト影 ≥ 8。supersample × AO はレイ数が乗算される点に注意。
- **決定論**: AO/影の乱数は `(pixel, sample)` のみからシードされ、**TBB スレッド数に依存しない**
  （同一入力は bit-identical）。
- **自己交差**: 二次レイは幾何法線方向へ、プライマリ光線長 `tfar` でスケールした適応 eps で
  オフセットする（遠景カメラでも shadow acne が出ない）。
- **例外安全**: 失敗時は `std::runtime_error`。CueMol 側で try/catch すること。

---

## 7. CueMol 統合の指針

- CueMol の内部シーンから **`umbreon::Scene` を直接構築**する（`.pov` を経由しない）。
  - 分子サーフェス／density map surface → `Scene::mesh`（頂点法線 + rgb+opacity 付き三角形メッシュ。
    `index` 省略で de-indexed スープも可）。
  - ball-and-stick / VdW → `Scene::spheres` / `Scene::cylinders`。
  - シルエットエッジ → `Cylinder{open=true}`（連結される）。bond/wireframe → `Cylinder{open=false}`。
  - section ごとの透過 → `triGroupId` / `Sphere::group` / `Cylinder::group` ＋ `veilGroups`。
- 画像は `render()` → `srgbEncode8()` → CueMol の画像バッファ／保存へ。
- カメラ・ライトは CueMol のビュー設定から `Camera` / `DistantLight` に変換する
  （POV-Ray 出力と一致させたい場合の換算は `docs/`／既存の POV パスを参照）。
- スレッド制御が必要なら `tbb::global_control` を呼び出し側で設定。

---

## 8. 既知の制限・今後

- **GI/二次反射なし**（single-bounce）。反射は `reflection * background` の近似のみ。
  立体感向上の AO 品質拡張（multi-scale / 距離減衰 / bent normal / multibounce /
  low-discrepancy / diffuse 減衰）は `aoFalloffPower` ほかで利用可（既定 OFF で bit-exact）。
  これらの設計は `docs/plans/ao-quality.md`、後段の diffuse GI cache は
  `docs/plans/surface-irradiance-cache.md` を参照。
- `albedo` / `normal` ほか AO AOV は `aoWriteAov` 有効時に `render()` で充填される
  （OIDN guide / surface irradiance cache の入力を兼ねる）。
- `render()` はフレームごとに Embree デバイス＋BVH を再構築する。対話的に多数フレームを
  回す用途では、デバイス／シーンを保持する `Renderer` ハンドル API の追加を将来検討
  （現状は静止画・バッチ用途を想定）。
- 法線反転などの入力前処理はライブラリでは行わない（CueMol 側でメッシュ構築時に対応）。
