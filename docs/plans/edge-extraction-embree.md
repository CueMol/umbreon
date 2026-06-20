# umbreon: エッジ抽出（silhouette/crease）の移管 — CGAL→Embree

ステータス: **計画済み・未着手**（後日実装）。本ドキュメントは設計合意の記録。

## Context（なぜ／何を）

現状、エッジ（silhouette/crease）抽出は **CueMol側**（`RendIntData.cpp` + `RendIntData_AABBTree.cpp`、CGAL AABB tree）で行い、検出・可視性判定・サーフェスへのクリッピングまで済ませて POV `edge_line()/edge_line2()` マクロを emit、umbreon がそれを cylinder としてパースして描画している。

目標：**エッジ抽出を umbreon の責務に移し、Embree で同等のことを行う**。これにより CueMol の rendering module から CGAL 依存を撤去でき、レンダリングパイプラインが umbreon に一本化される。

**確定済みの前提（ユーザ決定）**
1. **production = CueMol が umbreon を static link し、`umbreon::Scene` をメモリ上で直接構築して渡す**。`.pov/.inc` SDL 読み込みは PoC/bench 専用で production では使わない（CueMol integration ロードマップ option #1 と整合）。
2. **umbreon がエッジ抽出を丸ごと所有**：検出（silhouette/crease）＋可視性＋クリッピング＋cylinder/corner 生成。CueMol は CGAL パスと `edge_line` emission をやめる。
3. **連結性は weld で復元しない**：`Scene` が indexed surface mesh（共有頂点位置＋face indices）を運ぶ。CueMol は内部の indexed mesh（`m_pEgMesh` = `simplifyMesh` 出力、`m_verts`/`SEFace::iv1/iv2/iv3`）から直接詰める。umbreon の render 用 de-index mesh は per-corner 属性のための内部都合として温存。
4. **精度方針**：Embree コアは fp32 固定（`RTCRay`/`RTCHit` 全 float、`rtcore_ray.h:13-43`）で変更不可。**narrow-phase のみ fp64 へ逃がすエスカレーション**を用意（後述）。エッジ抽出 mesh の座標は **double で Scene に持たせる**（CueMol 内部は `Vector4D`=double）と端から端まで fp64 を保てる。

## スコープ境界（唯一の設計判断）

`simplifyMesh`（頂点 weld/簡略化、`RendIntData.cpp:1133` で `m_pEgMesh` を生成）は「エッジ抽出」ではなく mesh 前処理。**推奨：CueMol が既存 `simplifyMesh` を流用して簡略化済み indexed mesh を渡し、umbreon は検出以降を所有**（weld 再実装と weld 許容誤差リスクを完全回避）。CueMol が渡せない事情があれば umbreon が `simplifyMesh` を移植する（フォールバック）。

## アーキテクチャ（データフロー）

```
CueMol (production):
  surface mesh → simplifyMesh → indexed edge-mesh (double pos + faceIdx + per-vert normal)
  ＋ EdgeExtractionParams (creaseLimit, edgeWidth, rise, scale, cornerType, flags)
  → umbreon::Scene に格納し umbreon::render() 呼び出し

umbreon::render() 内、Embree build 後・primary render 前に:
  extractEdges(scene, edgeMeshBVH, params)
    1. 隣接構築: SEEdgeSet 相当（辺→接面≤2）
    2. 検出: face法線 + 視線で silhouette/crease 判定 → 候補エッジ集合 + corner点
    3. 可視性: Embree レイで各 corner/endpoint の可視判定
    4. クリッピング: 各エッジ線分 × surface の全交点 → fsec で可視区間に分割
    5. 生成: 可視区間を Cylinder(open, chained), corner を Sphere として scene に追加
  → 以降の描画は無改造（既存 cylinder/sphere パス、opacity1 グラデーション含む）
```

エッジ抽出用 Embree BVH は **indexed edge-mesh から専用に build**（render の de-index BVH とは別）。理由：primID→(iv1,iv2,iv3) が直接引け、`contains_id` 除外と可視性 id 照合が綺麗。抽出時点では sphere/cylinder は未生成なので干渉なし。

## 新規 umbreon API サーフェス

`src/scene.hpp`:
- `Mesh` はそのまま（render 用 de-index）。**新たに `struct EdgeMesh`** を追加：
  ```cpp
  struct EdgeMesh {                 // indexed, 抽出専用。座標は double で精度保持
    std::vector<Vec3d> positions;   // 共有頂点（Vec3d = double 版。新規 or 既存 double型）
    std::vector<Vec3d> normals;     // 共有 per-vertex 法線（rise方向・端点法線用）
    std::vector<uint32_t> indices;  // 3*faceCount, 共有位置インデックス
    std::vector<uint8_t> faceMode;  // CueMol nmode (MFMOD_MESH/SPHERE/NORGLN/MESHXX/...) 相当
  };
  ```
  （`Vec3d` が無ければ最小限の double 三要素型を追加。Embree へは broad-phase 用に float へ落として渡す。）
- `struct EdgeExtractionParams { float creaseLimit; float edgeWidth; float rise; float scale; int cornerType; bool detectSilhouette; bool detectCrease; ColorRGB edgeColor; uint16_t group; }`。
- `Scene` に `std::optional<EdgeMesh> edgeMesh;` と `EdgeExtractionParams edgeParams;` を追加。`edgeMesh` 未設定なら抽出スキップ（既存シーンは byte 不変）。
- `Camera` は変更不要（検出は direction/position/orthographic で足りる）。可視性レイ origin 距離は `EdgeMesh` の bounds から導出（mesh 外に出る十分な距離）。fidelity 必要時のみ任意で `viewDistance` を `EdgeExtractionParams` に追加可。

`src/render/embree_renderer.{hpp,cpp}`:
- `EdgeExtractor`（または free 関数 `extractEdges`）を追加。入力 `EdgeMesh + Camera + EdgeExtractionParams`、出力 `{std::vector<Cylinder>, std::vector<Sphere>}`。
- 既存の全交点 walk（front-to-back、`tnear` 前進、`embree_renderer.cpp:660-717`）を**汎用ヘルパに切り出して再利用**（`collectHitsAlongRay(scene, org, dir, tnear, tfar, excludePred) -> [(primID,t)]`）。フィルタコールバック不要。
- `RTC_SCENE_FLAG_ROBUST` 維持。

`src/umbreon.{hpp,cpp}`:
- `render()` 内で Embree build 後に `if (scene.edgeMesh) { auto [cyl,sph]=extractEdges(...); append to working geometry; }`。抽出結果は既存の cylinder/sphere レンダリング経路に流す。

## アルゴリズム移植 ＋ クエリ対応表

検出（`RendIntData.cpp` から移植、純トポロジ）:
- `calcNorm`（:1118）= cross 積 face 法線。
- 隣接 = 各面の3辺を `(minIv,maxIv)` キーで map に挿入し if1/if2 を埋める（`SEEdgeSet::insertEdge` 相当）。境界辺（if2 無し）は ridge → silhouette 扱い（`nmode != NORGLN` のとき）。
- silhouette: `checkSilEdge(view, n1, n2)`（:1099）= `dot(view,n1)*dot(view,n2)<0`。ortho は `view = -camera.direction`、persp は `view = vert - camera.position`。両端点 v1,v2 で判定（:1221/1230）。
- crease: `checkCrease(n1,n2,creaseLimit)`（:1109）= `acos(n1·n2) > limit`。
- `MFMOD_MESHXX` は強制無効、`MFMOD_NORGLN` ridge はスキップ（:1197/1209）。
- corner点（secpts）= silhouette 辺が使う頂点の一意集合（:1240-1274）。

可視性＋クリッピング（`RendIntData_AABBTree.cpp` を Embree へ）:

| CueMol 操作 (CGAL) | Embree イディオム |
|---|---|
| 頂点可視性 edge: `Segment(cam,vert)` all_intersections, `iv` 共有面除外（:138-162） | 有限レイ `org=cam,dir=(vert-cam)/L, tnear=eps, tfar=L-eps`、`collectHitsAlongRay`、primID の3 vert-id が `iv` を含むものを除外 → 非空なら遮蔽 |
| silhouette 可視性: `Ray(cam,vert)`（無限）（:167-205） | 同上だが `tfar=+inf`（無限レイの延長が本質。背後の遮蔽を数える） |
| 2辺コンテキスト: `contains_id`（:207-243） | 除外述語が iv1/iv2 両端 id を見る（`contains_id`:55-62 をそのまま移植） |
| エッジクリッピング: `Segment(pv1,pv2)` all_intersections → `fsec=|p-pv1|/L`（:266-339） | 有限レイ、各 hit で `fsec = t/L`、端点共有面を除外、`fsec` を sort/merge して可視区間生成 |

生成（`writeSilhLines`/`writeEdgeLines`/`writeCornerPoints` → 直接生成）:
- 可視区間ごとに `Cylinder{p0,p1, radius=edgeWidth*scale/2, open=true}`、`_sl_rise` 分を頂点法線方向にオフセット（`PovSilBuilder.cpp:163` と同じ）。
- `edge_line2` の端点 transmit グラデーション → `Cylinder.opacity1`（`scene.hpp:155`）に写像（既存の transp_0619 実装が消費）。
- corner（可視な secpt）→ `Sphere`（`ECT_*` cornerType に従う、`PovSilBuilder.cpp:201` 相当）。

## 精度戦略（fp32 → fp64 エスカレーション）

1. **v1（PoC含む）**: Embree 組み込み三角形（fp32 BVH）で broad-phase ＋ そのまま述語使用。必須緩和策：
   - `contains_id` 除外（coplanar 自己交差の主因を除去）
   - **法線方向オフセット `_sl_rise`** を**クエリ origin/線分にも適用**（grazing → 横断交差化、最も効果大・コスト0）
   - `tnear=+eps, tfar=L-eps`、`RTC_SCENE_FLAG_ROBUST`
2. **v2（必要時のみ）= Option A**: Embree は候補 primID 列挙だけに使い、各候補の **segment-triangle 交差を `EdgeMesh` の double 座標から double で再計算**（`fsec`・可視判定を fp64 化）。レイは保守的（eps 膨張）にして候補取りこぼし防止。coplanar 入射面は float AABB が線分を必ず含むので broad-phase で確実に訪問される（除外で落とす）。
3. **v3（最終手段）= Option B**: `RTC_GEOMETRY_TYPE_USER` で三角形登録、保守的 float AABB＋fp64 narrow-phase callback。Embree に float 頂点を一切渡さない。

PoC 計測で v1 の `fsec`/可視差分が許容外なら v2 へ。double 座標を Scene が持つことが v2/v3 の前提。

## PoC ファースト（caveat (b) を実装前に潰す、~1–2日）

production は直接 Scene だが、**PoC は bench/SDL を test 配管として流用**：`mesh2_reader.cpp:536` の `face_indices` を**破棄せず保持**して indexed `EdgeMesh` を組む（production API ではない、検証専用）。
- umbreon に **`isVertSilVisible` ＋検出**のみ実装し、既存の CGAL POV ベースライン（`docs/match` の `edge_line` マクロ）と比較：
  - 検出されたエッジ集合の一致、各 endpoint の可視ビット、クリップ `fsec` 値を diff。
- これで §精度 v1 の grazing リスクを実シーンで定量化し、v1 で足りるか v2 が要るかを大規模実装前に判定。
- 比較対象シーン：silhouette を持つ既存サンプル（例 `1ab0_scene5_transp2`、`scene2`）。

## 実装順（小さく独立検証可能）

1. **検出のみ（純 C++、Embree 不要）**: `EdgeMesh`/`EdgeExtractionParams` 型追加、隣接構築＋`checkSilEdge`/`checkCrease`/`calcNorm` 移植 → 候補エッジ＋corner を返す。単体テストで CueMol 検出結果と突合（bench で `EdgeMesh` を SDL から構築）。
2. **全交点 walk ヘルパ切り出し**: `collectHitsAlongRay` を `embree_renderer.cpp:660-717` から抽出、既存透過 walk をそれで書き直し（**bit-exact** を ctest で確認）。
3. **可視性（PoC 核）**: 専用 indexed edge-mesh RTCScene を build、`isVertSilVisible`（無限レイ＋`contains_id`）実装、CGAL POV ベースラインと diff（§PoC）。**ここが caveat (b) の判定ポイント**。
4. **クリッピング**: `Segment` 全交点 → `fsec` → 可視区間。必要なら精度 v2 へ。
5. **生成 ＋ render 配線**: 可視区間→`Cylinder(open)`、corner→`Sphere`、`opacity1` グラデーション、`extractEdges` を `umbreon::render` に組み込み。出力 PNG を CGAL POV ベースラインと PSNR/SSIM 比較。
6. **production API（CueMol 側）**: CueMol に umbreon::Scene exporter（Pov/LuxRender exporter に倣う）を追加、`simplifyMesh` 出力＋params を `EdgeMesh` に詰める。CGAL パス（`RendIntData_AABBTree.cpp`）と `edge_line` emission を撤去。
7. **精度エスカレーション（条件付き）**: PoC 結果次第で Option A（fp64 narrow-phase）。

## リスク / 注意

- **(b) fp32 grazing**（最大）: silhouette 辺は三角形辺上＝coplanar/grazing。v1 緩和策で「視覚的に区別不能」、v2 で「数値同等」。**必ず CGAL ベースライン diff で実測**（手順3で判定）。
- **double 起点の喪失**: `Vec3` は float。fp64 を活かすには `EdgeMesh` を double で保持（さもないと v2 が量子化済み入力からの再計算になり利得僅少）。
- **視点依存**: silhouette 検出は camera 必須。ortho/persp で view ベクトル定義が違う（:1217-1230）。可視性レイ origin は bounds 導出で mesh 外へ。
- **nmode 意味論**: `MFMOD_SPHERE/NORGLN/MESHXX` の分岐（ridge/force-no-edge）を移植。`SEEdge.bForceShow`（sphere mesh は常時表示、`RendIntData_AABBTree.cpp:289-293`）も再現。
- **退化クリップ結果**: CGAL は coplanar 時に `Segment` variant を破棄（:321-327）。Embree は端点近傍の偽 point hit を同様にフィルタ（除外＋eps）。
- **境界 mesh の一致**: 可視性 BVH と検出は**同一 indexed edge-mesh**から作る（primID→vert-id 整合）。

## 検証（end-to-end）

1. **検出単体**: bench で `EdgeMesh` 構築 → 候補エッジ集合を CueMol 出力（POV `edge_line` の端点）と突合（手順1）。
2. **bit-exact 退行**: `collectHitsAlongRay` 切り出し後、`task test`（`ctest --test-dir build --output-on-failure`）で既存透過テスト緑（手順2）。
3. **可視性/クリッピング diff（核心）**: PoC で per-edge 可視ビット・`fsec` を CGAL POV ベースライン（`docs/match`）と数値 diff（手順3-4）。
4. **画像一致**: `extractEdges` 込みレンダ PNG を CGAL POV ベースラインと PSNR/SSIM 比較（外部 Python/skimage。tool の `--compare` は PPM のみ）。silhouette サンプル（`1ab0_scene5_transp2` 等）で退行なきこと。
5. **新規単体テスト**: `tests/test_render.cpp` に検出（既知 mesh の silhouette/crease）＋可視性（遮蔽あり/なし）の合成シーンを追加。

## 主要ファイル

umbreon:
- `src/scene.hpp`（`EdgeMesh`/`EdgeExtractionParams` 追加、`Scene` 拡張、`Cylinder.opacity1/open`・`Camera` 再利用）
- `src/render/embree_renderer.{hpp,cpp}`（`collectHitsAlongRay` 切り出し:660-717、`extractEdges`、専用 edge RTCScene、`GeomKind`）
- `src/umbreon.{hpp,cpp}`（`render` 内 build 後・描画前に抽出を配線）
- `src/geom/mesh2_reader.cpp`（**PoC専用**: `face_indices`:536 を保持して `EdgeMesh` を組む）
- `tests/test_render.cpp`（検出・可視性の単体テスト追加）

cuemol2（production、手順6）:
- `src/modules/rendering/RendIntData.cpp`（検出ロジックの参照元 / production では `simplifyMesh` 出力を渡すのみに縮小）
- `src/modules/rendering/RendIntData_AABBTree.cpp`（CGAL 可視性/クリッピング — **撤去**）
- `src/modules/rendering/PovSilBuilder.cpp`（`edge_line` emission — production では停止）
- 新規 umbreon::Scene exporter（既存 Pov/LuxRender exporter に倣う）
