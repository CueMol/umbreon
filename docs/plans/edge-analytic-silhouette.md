# umbreon: `--edges` で ball-and-stick（sphere/cylinder）を解析シルエットで輪郭化

ステータス: **実装完了**（本ブランチ）。本ドキュメントは設計と実装の記録。

## Context（なぜ／何を）

screen-space の Freestyle STROKE パス（`--edges`、`render/stroke_edges.cpp` の
`applyStrokeEdges`）は、feature edge の抽出元が三角形メッシュ `scene.mesh` ただ1つ
（`extractMeshFeatureEdges`）だった。CueMol→POV 変換では ribbon=`mesh2`→`scene.mesh`、
ball-and-stick の原子=`sphere`→`scene.spheres`、ボンド=`cylinder`/`edge_line`→
`scene.cylinders` になるため、**`--edges` をかけても ribbon にしか輪郭が付かず、
ball-and-stick の sphere/cylinder には一切付かない**という問題があった。

調査の結果、可視性（QI 遮蔽判定）そのものは元から全 BVH（mesh + spheres + cylinders、
全 group）に対し global に機能していた（単一の committed `RTCScene` に全プリミティブが
attach 済み）。不足していたのは **解析プリミティブの edge 生成**だけだった。

目標：**`--edges` がデフォルトパラメータのまま mesh / sphere / cylinder すべてを輪郭化し、
原子リングがボンドに埋まる部分などの接合部 hidden-line も真の遮蔽で正しく処理する**。

## 方針（解析シルエット → stroke pipeline へマージ）

sphere/cylinder の **解析 n·v シルエット**（CueMol の "edge" と同じ silhouette。`--obj-edges`
が既に持つ数式）を `FeatureSeg`（nature=Silhouette）として生成し、mesh edge と同じ seg
リストへマージする。これにより mesh edge と同一の chaining + QI hidden-line + per-section
リボンスタイリングをそのまま得る。`--obj-edges`（解析エッジを 3D cylinder として焼き込む
別経路）の発見的 union-clip より正確な、真の image-space hidden-line になる。

## 設計の要点

### 1. 共有コア `render/analytic_silhouette.{hpp,cpp}`
`mesh_feature_edges` が `--edges`/`--obj-edges` 両方に共有されているのと同じ構図で、解析
silhouette の emitter（sphere horizon ring / cylinder 側線 / end-on cap rim）を
`object_space_edges.cpp` の匿名 namespace から **数式そのまま移設**し、中立な
`AnalyticLoop`（順序付き輪郭頂点列）を返す形にした。

- `emitAnalyticSilhouettes(scene, cam, N, raise, circumscribe, out)` … 全 sphere→cylinder を
  index 順に走査（`fromEdgeMacro` の baked outline は skip）。
- `appendAnalyticFeatureSegs(scene, cam, N, raise, nodeBase, segs)` … 上記ループを Silhouette
  `FeatureSeg` へ変換。各ループに連続 node id ブロックを専有割当（sphere ring/cap は閉ループ、
  cylinder 側線は独立 1-seg）。`face0=face1=-1`・`excludeFaces` 空・`nrm=0`。
- `--obj-edges` 側は `emitAnalyticSilhouettes(..., circumscribe=false)` を呼び、同一順・同一値で
  `RawSeg` へ flatten するだけ。**byte-identical**（`tests/test_object_space_edges.cpp` の幾何
  アサーションで担保）。

### 2. リング circumscription（自己遮蔽=破線の防止）
sphere ring / cap circle の弦は中点が球内へ sagitta `r(1-cos(π/N))`（N=48 で ≈0.2%r）沈むため、
QI サンプルレイが近側の球面を貫いて**偽の自己遮蔽（破線化）**を起こしうる。stroke 経路では
リング半径を `1/cos(π/N)` 倍して多角形を contour に外接させ、頂点を面外・弦中点を面上に乗せる
（レイが再侵入しない）。拡大は N=48 で約0.2%（サブピクセル不可視）、実遮蔽体の手前には出ない。
cylinder 側線は球面/円筒面上の直線（全点 tangent）なので不要。`circumscribe=false`（obj-edges）は
従来挙動。

### 3. QI 自己遮蔽は grazing/coplanar フィルタで（mesh face id 非依存）
解析 seg は `excludeFaces` 空・`face0/1=-1`。QI 遮蔽フィルタ（`embree_renderer.cpp`
`excludeFaceFilter`）の self-FACE 除外は mesh 限定だが、**grazing（`|dir·Ng| ≤ grazeCosEps`）と
coplanar 除外は geomID 非依存**で sphere(`SPHERE_POINT`)/cylinder(`ROUND/CONE_LINEAR_CURVE`)の
`Ng` にも効く。silhouette 点の eye-ray は自表面に tangent なので自己 hit が grazing 除外される
（mesh の smooth n·v==0 contour と同じ機構）。
mesh 無しシーンでは `coplanarEps=0.2*meanEdge` が 0 になるため、`umbreon.cpp` で解析プリミティブの
平均半径から非ゼロ化（mesh 有りシーンは不変＝byte-identical）。

### 4. 解析シルエットは常に「遮蔽境界」として使い、描画のみゲートする
解析プリミティブの silhouette は**真の occluder boundary**。stroke パスの可視性は
per-ViewEdge の QI **多数決**なので、mesh edge が遮蔽体の輪郭で分割（2D crossing の T-vertex）
されないと、ribbon edge の「sphere/cylinder 背後に回る部分」を多数決が「可視」に倒して
**リーク（ball-stick の上に ribbon の線が描かれる）**を起こす。

したがって解析 seg の**append は `se.silhouette` のみでゲート**し（常に可視性=交差分割に参加）、
`--stroke-analytic`（`se.analytic`）は**ラスタライズ（描画）だけをゲート**する。chain は node id が
mesh と disjoint なので「解析由来 chain」は `s0.v0 >= fm.nodeCount` で判別でき、`--stroke-analytic off`
の時だけ描画を skip する。

結果:
- `--stroke-analytic on`（既定）… ball-stick も輪郭化＋正しい hidden-line。
- `--stroke-analytic off` … ball-stick は輪郭なし、**但し ribbon の隠線は正しい**（リークなし）。

### 5. 決定性
解析 emission は `scene.spheres`→`scene.cylinders` を index 順、node id を逐次採番。chaining・
交差パス・per-chain QI は単一スレッド、ラスタライズのみ tile-deterministic。マージ順は固定で、
TBB スレッド数非依存・bit 一致を維持（`tests/test_stroke_analytic.cpp` の determinism チェック）。

## CLI

| オプション | 既定 | 効果 |
|---|---|---|
| `--edges <on/off>` | off | stroke edge パスの master gate（変更なし） |
| `--stroke-analytic <on/off>` | **on** | sphere/cylinder（ball-stick）の輪郭を**描画**。off でも遮蔽境界には使う（隠線は正しいまま、ball-stick が無輪郭になるだけ） |
| `--stroke-analytic-segments <int>` | 48 | sphere ring / cap circle の分割数（`--obj-edge-segments` に対応） |

`--edges on` 単体（追加フラグ不要）で全プリミティブが既定 silhouette スタイルの輪郭を得る。

## 変更ファイル

- 新規: `render/analytic_silhouette.{hpp,cpp}`（共有コア＋stroke adapter）
- `render/stroke_edges.cpp`（`applyStrokeEdges`: 解析 seg のマージ＋描画ゲート）
- `render/object_space_edges.cpp`（共有コア利用へ。byte-identical）
- `umbreon.cpp`（mesh 無しシーンの `coplanarEps`）
- `render/render_types.hpp`（`StrokeEdgeOptions::analytic` / `analyticSegments`）
- `bench/cli.{hpp,cpp}` / `bench/main.cpp`（`--stroke-analytic[-segments]` 配線）
- `CMakeLists.txt` / `tests/CMakeLists.txt`（新ソース・新テスト登録）
- 新規: `tests/test_stroke_analytic.cpp`

## テスト（`tests/test_stroke_analytic.cpp`, 25 checks）

- 単体: 解析 FeatureSeg 構造（sphere→閉リング/cylinder→側線2本、node id 採番、Silhouette/group/
  face=-1/nrm=0）と chaining、circumscription（頂点が面外・弦中点が面上）、`fromEdgeMacro` skip。
- 統合（`umbreon::render`）: 自由球の**連続輪郭**（破線なし）、**cross-primitive 隠線**（球が手前の
  球に隠れる）、**`--stroke-analytic off` でも mesh edge が球背後で隠れる**（リーク再発検知）、決定性。

## 既知の限界（非ブロッカー）

- `computeEdgeCrossings` は O(N²)。解析 seg で N が増える（sphere 1 個 ≈ `segments` seg）ため、
  大規模 ball-and-stick シーンでは hot spot 化しうる。正確性ではなく性能の問題。必要なら screen-grid
  バケットを follow-up。
- bond は swept-sphere の丸端（`ROUND_LINEAR_CURVE`）だが解析 emitter は直線側線（+ end-on 時のみ
  cap 円）。丸端の小アークが未被覆になりうるが通常は原子に埋もれる（`--obj-edges` と同等の妥協）。
