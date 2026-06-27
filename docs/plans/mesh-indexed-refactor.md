# umbreon: Mesh を indexed 化する refactoring プラン

ステータス: **実装済み（2026-06-27）**。本ドキュメントは設計合意の記録。実装は
index-optional `Mesh`（`scene.hpp`）＋ load 時 weld（`mesh2_reader.cpp::buildIndexedBlock`、
描画用は厳密属性 dedup・トポロジ用は `render/mesh_weld.hpp` の位置クラス `posClass`）＋
Embree indexed build（`scene_build.cpp`）＋ 特徴線抽出の posClass 消費（`mesh_feature_edges.cpp`、
posClass 無し時は自前 weld へ fallback）。検証は `tests/test_mesh_indexed.cpp`（合成）と
`tests/test_edge_regression.cpp`（実データ: edge_tube1/ribbon1/ribbon2 + 1ab0_scene1 を
edges-on で indexed vs de-indexed bit-identical 比較）。ribbon1 で頂点 ~6x 削減。

> 注意（スコープ外）: この refactoring は **メモリ効率・データ構造の整理** が目的であり、
> grazing twist で外形線が欠ける **cusp バグとは無関係**（直らない）。cusp は別件
> （`computeCusps` 移植）。混同しないこと。

## Context（なぜ）

現在の `Mesh`（`src/umbreon/scene.hpp`）は **de-indexed = 三角形スープ**:
- `positions` / `normals` / `colors` はサイズ `3 × triangleCount`、三角形 i = `[3i, 3i+1, 3i+2]`。
- 共有頂点インデックスを持たない。頂点共有トポロジーはメモリ上どこにも無い。
- Embree へも自明 index（`idx[t*3+k] = 3t+k`、`render/scene_build.cpp`）で渡しており、
  index 機構を使っていない。

入力の POV mesh2 は元々 indexed（`vertex_vectors` + `face_indices`）だが、
`mesh2_reader.cpp::deindexBlock()` が読み込み時にスープへ展開して捨てている。
特徴線抽出は毎回、位置溶接でトポロジーを復元している（`mesh_feature_edges.cpp` step 1）。

問題点:
1. **メモリ非効率**: スープは頂点を最大 6 倍重複保持（後述の実測）。
2. Embree に **proper な indexed mesh を渡せていない**（indexed パスの最適化を活かせない）。
3. 特徴線抽出で **毎回溶接** している（load 時 1 回で済むはず）。

## 実測（ribbon1, 52400 tris, 単一 mesh2 ブロック）

| 表現 | 頂点数 | 備考 |
|---|---|---|
| 現在（三角形スープ） | 157200 | `3 × 52400` |
| POV ネイティブ index | 54520 | ただし 1060 本の triangle strip に分断（巻きごと、巻き間は非連結） |
| **厳密重複排除 (位置+法線)** | **27254** | レンダリング用に妥当な代表 |
| 厳密重複排除 (位置のみ) | 26674 | トポロジー用 |
| 1e-4 溶接 (位置のみ) | 26636 | 現在の特徴線抽出が使う |

要点:
- **巻き境界の重複頂点は「厳密に」同値**（厳密位置 unique 26674 ≈ 1e-4 溶接 26636、差 38）。
  → 厳密な属性込み重複排除で巻き間接続が復元でき、**かつ座標を動かさないので描画は bit-identical を維持できる**。
  1e-4 が効くのは残り 38 点（＝本物の近接接触＝非多様体候補）だけ。
- 法線で分割が必要な頂点は **約 580 個のみ**（27254 − 26674、＝ハードエッジ）。
- 色は per-corner で **17 種の texture id**。dedup キーに色も含める必要がある。
- ＝「全部 triangles にバラす必要はない」（重複排除＋属性差のみ分割）の方針が正しい。

## 提案する設計

CueMol2/POV の mesh を load 時に **1 回だけ溶接して indexed mesh を構築**し、Embree へ直接渡す。
属性が食い違う頂点だけ別 ID に分割する（全展開しない）。

### 落とし穴: レンダリングとトポロジーで併合基準が異なる

- **レンダリング/Embree** → `(位置, 法線, 色)` で併合（ハードエッジ・色境界で分割）≈ 27254 頂点。
- **特徴線抽出 (crease/silhouette/border)** → **位置のみ**で併合（ハードエッジでも両面が同一頂点を
  共有しないと crease が border に誤判定される）≈ 26674。

→ 属性分割した index をそのまま抽出器に渡すと crease が壊れる。

**対策**: 1 回の溶接で **「描画用 index（属性分割）」＋「頂点 → 位置クラス写像」** を同時生成。
抽出器は位置クラスで隣接を見る。これで再溶接も不要になる。

### bit-identity と許容誤差

- 描画用 dedup は **厳密属性一致**（位置・法線・色のビット一致）で行う → 座標を動かさず描画 bit-identical。
- 1e-4 トレランスは「トポロジーが近接 38 点を橋渡しするか」だけの局所判断に縮小し、**描画とは切り離す**
  （現状はこの近接併合が非多様体 72 本の原因。トポロジー専用の判断にする）。

## 影響範囲（regression 面）

- `Mesh`（`scene.hpp`）に index 配列（と任意で位置クラス写像）を追加。`positions/normals/colors` は
  per-vertex 化、`triangleCount` の定義変更。
- `mesh2_reader.cpp`: `deindexBlock()` を indexed 構築へ（per-corner 法線/色を属性キーに）。
- `scene_build.cpp`: 自明 index → 実 index。per-corner 参照箇所の見直し。
- `mesh_feature_edges.cpp`: 自前の位置溶接を廃し、load 時の位置クラス写像を使用。
- 直接 `positions[3i+k]` を仮定している全 consumer の確認（`triMaterialId`/`triGroupId` は per-tri のまま）。
- **検証**: 既存の byte-identity テスト（canonical 比較、`tests/test_render.cpp`）で描画不変を担保。
  厳密属性キーで併合する限り維持されるはず。

## 関連

- 溶接の現行実装と目的: `render/mesh_feature_edges.cpp` step 1 のコメント、`docs/api/libumbreon.md` §4.3。
- 本件は cusp（外形線欠け）とは独立。cusp は Freestyle `computeCusps` 移植が別途必要。
