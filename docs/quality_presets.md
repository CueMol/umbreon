# レンダリング品質プリセット設計ガイド（client UI 向け）

`RenderOptions` のフィールドは多数あり、個別に見ると全体像がつかみにくい。このドキュメントは
それらを **少数の「軸（グループ）」に分解**し、各軸に **3〜4 段階のプリセット**を定義することで、
client（CueMol）側が UI のプリセットとして扱えるようにするための設計指針である。

- **product は libumbreon**。client は `.pov` を経由せず、`umbreon::Scene` と `umbreon::RenderOptions`
  を直接組み立てて `render()` を呼ぶ。したがって本書のプリセットは **`RenderOptions` のフィールド値**を
  正（Single Source of Truth）とし、CLI フラグ（`umbreon_cli`）は対応の参考として併記する。
- API 全体は [api/libumbreon.md](api/libumbreon.md)、AO/pt1 の詳細チューニングは
  [umbreon_cli.md](umbreon_cli.md) / [pt1_tuning.md](pt1_tuning.md) を参照。

---

## 0. 設計の考え方 — 品質は 1 本のダイヤルではなく「軸の合成」

品質は単一のスライダーではなく、**独立した複数の軸**の組み合わせで決まる。client は
「各軸ごとのドロップダウン（詳細モード）」または「全軸をまとめて動かす単一プリセット
（かんたんモード）」のどちらでも構成できる（[§6](#6-複合プリセット単一ドロップダウン用) に複合例）。

| 軸 | 内容 | 段階 | 排他・注意 |
|---|---|---|---|
| **A. 基本画質** | supersample / adaptive AA | low / medium / high / ultra | 常時。コストは `supersample²` |
| **B. 奥行き・陰影法** | なし / AO / GI(pt1) | 各 low / medium / high | **AO と GI は排他**（両方とも「凹は暗く凸は明るく」を担う） |
| **C. 影** | shadows（ハード/ソフト） | off / low / medium / high | メッシュのみに落ちる（後述） |
| **D. エッジ（NPR）** | stroke / object-space | off / 2 方式 | **品質段階ではなくスタイルの切替**。2 方式は相互排他 |
| **E. その他** | 透過 / specular / denoise 等 | 概ね既定固定 | [§5](#5-その他の設定候補その他の軸) 参照 |

各プリセットの**実測コスト**（このリポジトリの分子表面シーンで計測）は
[付録B](#付録b-実測コスト参考) にまとめてある。

> **最重要の前提**: 軸 B の AO と GI(pt1) は **同じ目的（陰影による立体感）を別方式で達成する
> 代替**である。UI 上は「陰影法: なし / AO / GI」の **単一セレクタ**にし、選んだ方式の中で
> low/medium/high を選ばせるのが最もシンプル。両方を同時に有効化しても意味がなく、
> 一部の高速化オプション（coarse AO・adaptive AA）は GI と併用不可（後述）。

---

## 1. 軸A: 基本画質（supersample / adaptive AA）

全モードに効く土台。**きれいさに一番効くレバーは `supersample`**（`w*ss × h*ss` で描画して
linear 平均。AO/影/GI のモンテカルロノイズもこの平均でデノイズされる）。コストは `ss²`。

| フィールド | low | medium | high | ultra |
|---|---|---|---|---|
| `supersample` | 1 | 2 | 3 | 4 |
| `aaMode` | 1 (adaptive) | 1 (adaptive) | 0 (grid) | 0 (grid) |
| `aaDepth` | 3 | 3 | 0 (=ss) | 0 (=ss) |
| `aaThreshold` | 0.1 | 0.1 | 0.1 | 0.1 |

- **adaptive AA**（`aaMode=1`）は「境界ピクセルだけ細かく評価、平坦部は複製」する決定論的
  AA。`supersample=1` でも `aaDepth=3` で境界を 3×3 相当までシャープにでき、`ss=3` グリッドの
  約 1/4 のコストで近い縁品質になる（平坦部は `ss=1` と同一）。低〜中プリセットの現実解。
- ただし **adaptive AA は GI と併用不可**（`gi` 有効時は警告を出して grid にフォールバック）。
  GI プリセットを選ぶ場合、基本画質は grid（`aaMode=0`）で `supersample` を上げること。
- **`--edges`（NPR）を使う場合は `supersample≥3` を維持**する。ストロークの AA は supersample
  由来で、`aaDepth` はストロークには効かない。

CLI: `--supersample <int>` / `--aa grid|adaptive` / `--aa-depth <int>` / `--aa-threshold <float>`。

---

## 2. 軸B: 奥行き・陰影法（AO と GI(pt1) は排他）

分子表面で「凹（ポケット・溝）は暗く・凸は明るく」という立体感を与える軸。既定
（`RenderOptions` を素で構築）は **両方 OFF** で、POV 忠実なフラットなローカルシェーディング。

![陰影法の比較: Flat（陰影なし）/ AO / GI(pt1）](images/depth_methods.png)

*左: Flat（AO/GI なし = 平坦）。中: AO medium（凹部を強く暗化）。右: GI pt1 medium（柔らかい奥行き）。
UI では「なし / AO / GI」を単一セレクタで選ばせ、選んだ方式の中で low/medium/high を選ぶ構成を推奨。*

### 2a. AO プリセット（low / medium / high）

AO は「メッシュ＋実プリミティブ（VdW 原子球・bond シリンダ）」の遮蔽を評価して陰影を付ける。
NPR の outline 装飾（シルエットエッジ）は暗くならない。

| フィールド | low（プレビュー） | medium（標準） | high（高品質） |
|---|---|---|---|
| `aoSamples` | 16 | 64 | 256 |
| `aoResDiv` | -1 (out) | -1 (out) | 0 (inline) |
| `aoLowDiscrepancy` | true | true | true |
| `aoMultiScale` | false | true | true |
| `aoBentNormal` | false | true | true |
| `aoFalloffPower` | 0 (binary) | 0 (binary) | 0 (binary) |
| `aoDiffuseFactor` | 1.0 | 1.0 | 1.0 |
| `aoIntensity` | 1.0 | 1.0 | 1.0 |
| `aoMultibounce` | false | false | false（淡色凹部の黒潰れが気になれば true） |
| `aoDistance` | **client が設定**（下記） | 同 | 同 |

**根拠と実測（ao_test1 700² ss=3）**: high は OpenGL GTAO 相当の奥行きが出る検証済みレシピ
（`aoDiffuseFactor=1.0` + `aoMultiScale` + `aoBentNormal` + binary falloff、256spp inline ≈ 53s）。
medium はその見た目を保ちつつ coarse-grid（`aoResDiv=-1`）+ 64spp + low-discrepancy で ≈ 2.3s に
落とした現実解。low は 16spp まで削ったプレビュー用（coarse-grid のセル斑が出うる）。
詳細は [umbreon_cli.md 奥行き節](umbreon_cli.md)。

**client 統合の必須事項**:

1. **`aoDistance` はライブラリが自動計算しない**（既定 `1e20` = 実質無限）。client が
   **シーンの bounding-box 対角の 0.5〜0.85 倍**を渡すこと。分子スケールでは**大半径ほど強い**
   （短半径は近傍に occluder が無く効かない）。
2. **`aoDiffuseFactor=1.0` が立体感の本命レバー**。AO は既定で ambient 項しか減光しないが、
   CueMol 既定ライティングはエネルギーの大半が direct light にあるため、これを 0 にすると
   AO は AO OFF と数%しか変わらず陰影がほとんど見えない。強度を弱めたいときは 1.0 → 0.5 等に
   下げる（`aoIntensity` は AO OFF 側の減光率で、>1 は過暗化）。
3. `aoResDiv=-1`（"out" = 出力解像度で 1 回 gather）は **`supersample≥2` のときだけ効く**。
   `supersample=1` では inline に退化する（軸 A で ss を上げる前提）。

CLI: `--ao-samples` / `--ao-res out` / `--ao-ld on` / `--ao-multiscale on` / `--ao-bent-normal on`
/ `--ao-diffuse 1.0` / `--ao-distance <world>`。

### 2b. GI(pt1) プリセット（low / medium / high ＋ reference）

pt1 は per-pixel の 1〜多バウンス path-traced gather。フラットな ambient を**遮蔽を考慮した
本物の irradiance に置き換える**。AO と違いデノイザ（OIDN）を内蔵するので低 spp でクリーン。

> **重要 — 品質軸と見た目軸を分けること**: pt1 の設定には性質の異なる 2 種がある。
> - **品質軸**（`pt1Spp` / `pt1GatherDiv`）… 上げると **同じ絵に収束**する。ノイズが減るだけで
>   明るさ・色・コントラストは変わらない。**これが low/medium/high の中身。**
> - **見た目軸**（`giBounces` / `giIntensity` / ambient 配分 / `pt1SkyMode`）… **絵そのものを
>   変える**。とくに `giBounces` を増やすと光がポケットに回り込んで**全体が明るくなり、奥行き
>   コントラストが下がる**（[2b-look](#giの見た目軸バウンス-強度) 参照）。
>
> 品質段には **品質軸だけを入れ、見た目軸（とくに `giBounces`）は全段で固定**する。混ぜると
> 「high ＝ 高品質」ではなく「high ＝ 明るい別の絵」になってしまう（実測は
> [付録B](#付録b-実測コスト参考) 参照。`giBounces` を 1 に固定すると low/medium/high/reference の
> foreground 平均輝度は 170.0〜170.5 でほぼ一致し、差はノイズのみになる）。

![giBounces は見た目軸: spp 固定でバウンスだけ変えると明るさが変わる](images/gi_bounces_look.png)

*すべて 64 spp（品質は同じ）。`giBounces` を 1→2→3 と増やすと光がポケットに回り込み、foreground
平均輝度が 171→185→190 と上がって奥行きコントラストが下がる。これは「品質」ではなく「見た目」の
変化なので、品質段には入れず全段で固定する（分子表現の既定推奨は 1）。*

**品質プリセット（`giBounces` などの見た目軸は全段共通で固定）**:

| フィールド | low（下書き） | medium（標準） | high | reference（参照・非対話） |
|---|---|---|---|---|
| `gi` | true | true | true | true |
| `giIntegrator` | 2 (pt2, 既定) | 2 | 2 | 2 |
| `pt1Spp` | 8 | 32 | 64 | 256 |
| `pt1GatherDiv` | -1 (out) | -1 (out) | -1 (out) | 0 + `pt1HalfRes=false`（full） |
| `pt1Ld` | true | true | true | false（256spp では不要） |
| `pt1Denoise` | true | true | true | true |
| `pt1EdgePatch` | true | true | true | true |
| `giBounces` | **全段共通の固定値（既定 1）— 品質軸ではない** ↓ | | | |

<a id="giの見た目軸バウンス-強度"></a>
**GI の見た目軸（バウンス・強度）— 品質段とは独立に、全段で同じ値**:

| フィールド | 既定 | 効果 |
|---|---|---|
| `giBounces` | **1** | 1 = 奥行きコントラスト最大（分子表現の既定推奨）。2 = 深いポケットの黒潰れを物理的に埋めるが**全体が明るく平坦寄り**に。3 以上はさらに明るく（reference でも同じ値にすること） |
| `giIntensity` | 1.0 | 間接光の一律ゲイン。>1 で陰も持ち上がるが凸部が飽和しうる |
| ambient 配分 | `.pov` は `_amb_frac` | GI が gather するエネルギー量。client は `scene.ambientColor` で与える（下記） |
| `pt1SkyMode`/`pt1SkyRadiance` | 0/白 | sky の向き・色（形状感の補助） |

**現行 CLI `--quality` との対応と注意**:

| 本書（品質段） | 現行 `--quality` | 現行が変える `giBounces` |
|---|---|---|
| **low** | `draft` | 1 |
| **medium** | `high` | **2** |
| **high** / **reference** | `ultra` | **3** |

> ⚠️ **現行 CLI `--quality` は品質段ごとに `giBounces` も 1/2/3 と変えており、上記の原則に反する**
> （段を上げると品質だけでなく明るさ・コントラストも変わる = 別の絵になる）。client プリセットでは
> `giBounces` を固定し、`pt1Spp` だけを段で変えること。CLI 側も `--gi-bounces` を `--quality` から
> 切り離す修正が望ましい（[付録](#付録-renderoptions-フィールド分類早見表) の後続作業案）。
>
> low/medium/high は **出力解像度で gather → joint-bilateral upsample** するため、旧フルグリッド
> プリセットと見た目同等を約 1/12 のコストで得る（1024spp 収束参照と user 評価済み）。reference は
> **品質段の収束先**なので、`giBounces` は low/medium/high と同じ値にする（違えると収束参照に
> ならない）。

**client 統合の必須事項**:

1. **`scene.ambientColor` に ambient エネルギーを載せること**。pt1 は「フラット ambient を
   遮蔽考慮の gather で置き換える」ので、ambient エネルギーが 0 だと**ほぼ no-op**（フラット描画と
   同色）になる。`.pov` パスでは harness が `_light_inten*_amb_frac` を `ambientColor` に流し、
   direct light を対応して下げてトータル輝度を保つ。CueMol でも **direct light とのエネルギー
   配分**（例: 半分を ambient へ）を `ambientColor` に設定し、direct light 側を対応して下げる。
   `giEnvIntensity`（既定 1.0）は sky/ground miss 項に対する `ambientColor` の乗数。
2. **`giMaxDistance` はライブラリが自動計算する**（既定 0 = auto ≈ シーン対角）。cache 経路の
   ローカル寄りコントラストを真似たいときのみ明示（`0.1×対角` 程度）。
3. **`envLights`（環境ドームライト）と pt1 は併用しない**（sky を二重計上する。警告あり）。
   GI の sky は `pt1SkyMode` / `pt1SkyRadiance` / `giEnvIntensity` で制御する。
4. `giSamples` や `giAccuracy` 等は **cache 経路（`giIntegrator=0`）専用**。pt1/pt2 では読まれない。

CLI（品質段を明示フラグで・バウンスは固定）:
`--integrator pt1 --indirect-res out --gi-bounces 1 --pt1-ld on --spp {8|32|64}`。
一括の `--quality draft|high|ultra` は **`--gi-bounces` も段ごとに変える**ため、見た目を揃えたい
用途には非推奨（上の ⚠️ 参照）。

### 2c. AO と GI どちらを既定にするか

- **GI(pt1) 推奨**: デノイザ内蔵で低 spp でもクリーン、拡大してもざらつかず、物理的に正しい。
  奥行き表現が目的なら AO より速く仕上がる（実測: GTAO full 53s に対し pt1 draft 1.5s）。
- **AO**: cache も denoiser も無く supersample 平均頼みなので高サンプルが要るが、局所コントラスト
  （接触影）を短半径で強調しやすい。GI が重い/不要な軽量用途向け。
- UI では「陰影法: なし / AO / GI」の単一セレクタ + 選択後に low/med/high、を推奨。

---

## 3. 軸C: 影（shadows）

ライトからの可視性による落ち影。**メッシュにのみ落ちる**（球・シリンダには落ちない）。
奥行き軸（AO/GI）とは独立に足せる。

| フィールド | off | low（ハード） | medium（ソフト） | high（滑らかソフト） |
|---|---|---|---|---|
| `shadows` | false | true | true | true |
| `shadowSamples` | — | 1 | 16 | 32〜48 |
| `lightRadius`（度） | — | 0（ハード） | 3 | 5〜6 |

- `lightRadius > 0` でエリアライト＝ソフト影（penumbra）。度指定でシーン非依存。大きいほど
  penumbra が広くノイジーになるので `shadowSamples` を増やす。
- コスト ≈ `supersample² × ライト数 × shadowSamples` の二次レイ。

CLI: `--shadows on --shadow-samples N --light-radius <deg>`。

---

## 4. 軸D: エッジ（NPR）— スタイル切替（品質段階ではない）

シルエット/輪郭線。**品質の高低ではなく「描くか / どの方式か」の選択**。2 方式は相互排他
（同時 true で `render()` が例外）。

| 方式 | フィールド | 特徴 | 品質ノブ |
|---|---|---|---|
| なし | 両 `enable=false` | 既定 | — |
| **A: ストローク** | `strokeEdges.enable=true` | Freestyle 風。可変幅リボン | `strokeEdges` 内の太さ/リサンプル/QI |
| **B: オブジェクト空間** | `objectSpaceEdges.enable=true` | 解析的輪郭を細い円柱で描画 | `objectSpaceEdges.segments`（球/キャップ分割）等 |

スタイル（色・幅・クラス別 on/off、セクション別上書き）は `EdgeStyle` / `Scene::groupEdgeStyle` で
指定。詳細は [libumbreon.md §4.9](api/libumbreon.md) と [edges_screen_vector.md](edges_screen_vector.md)。

---

## 5. その他の設定候補（その他の軸）

軸として立てるほどではないが client が触れる/固定すべきフィールド。

| フィールド | 既定 | 位置づけ |
|---|---|---|
| `transparency` | true | 透過 walk（front-to-back）。CueMol の section alpha を出すなら on 維持。**機能トグル**（品質段階ではない） |
| `transparentBackground` | false | 出力 alpha = 累積被覆（合成用）。**用途トグル** |
| `maxTransparentLayers` | 256 | 透過ヒットの安全上限。通常触らない（固定） |
| `specularScale` | 1.0 | マテリアル specular 量の一括倍率。**スタイル**（カートゥーンは 0 = matte 等） |
| `denoiser` ほか | None（cache-GI 時 atrous） | **cache 経路（`giIntegrator=0`）の最終色デノイズ**。pt1 は `pt1Denoise` が担うので触らない。固定でよい |
| `envLights` ほか | 0（off） | 環境ドームライト。AO/GI とは**別方式の代替ライティング**。pt1 とは併用不可（二重計上）。通常 off |
| `pt1SkyMode` / `pt1SkyRadiance` | 0(uniform)/白 | GI の sky モデル。上方向を明るくして形状感を足す（`gradient` + `aoGroundColor`）。**ライティングのスタイル** |
| `pt1Seed` | 0 | 決定論シード。固定 |

> **cache-GI（`giIntegrator=0`）について**: `gi=true` の既定インテグレータは **pt2**（2026-07 に
> pt1 から昇格）。irradiance cache は experimental・凍結で、明示指定したときだけ走る。
> **client 向けプリセットは既定の pt2（`giIntegrator=2`）に統一**することを推奨（pt1=1 は
> 凍結された回帰アンカーで、area light / emissive GI / traced reflection を読まない）。
> cache 固有ノブ（`giSamples` / `giAccuracy` / `giGradients` / `giOutlierReject` / `denoiser` 等）は
> pt1/pt2 プリセットでは無視され、UI に出す必要はない。

---

## 6. 複合プリセット（単一ドロップダウン用）

「かんたんモード」で 1 つのドロップダウンから全軸を動かしたい場合の推奨バンドル。**陰影法
（AO / GI）は直交する選択**として残し、レベルで基本画質・陰影の品質・影を一括で上下させる。

| レベル | 軸A 基本画質 | 軸B（GI 選択時） | 軸B（AO 選択時） | 軸C 影 |
|---|---|---|---|---|
| **Preview** | ss=1, adaptive, depth 3 | GI low | AO low（ss=1 なので coarse は inline） | off |
| **Standard** | ss=2, adaptive/grid | GI medium | AO medium | low |
| **High** | ss=3, grid | GI high | AO high | medium |
| **Publication** | ss=4, grid | GI reference | AO high | high |

- GI を選ぶレベルは **軸 A を grid 固定**（adaptive は GI 非対応）。AO を選ぶレベルは adaptive 可。
- `Publication` の GI は reference（256spp / 数分）で、静止画の書き出し専用。対話プレビューには
  Preview/Standard を割り当てる。
- **`giBounces` はレベルで変えない**（見た目軸なので全レベル共通の固定値。既定 1）。レベルが
  変えるのは品質軸（`pt1Spp` / 解像度）と影のみ。→ Preview→Publication で「明るさ・コントラストは
  同じまま、ノイズと縁だけが良くなる」挙動になる（[§2b](#2b-gipt1-プリセットlow--medium--high--reference) 参照）。
- 影は分子表現では必須ではない（AO/GI で立体感が出る）。既定 off でも良い。

---

## 7. CueMol 統合の落とし穴（チェックリスト）

client が `Scene`/`RenderOptions` を直接作る際に、**ライブラリが自動でやってくれないので client が
必ず設定/計算すべき**もの:

1. **`aoDistance`** — AO 使用時。bbox 対角の 0.5〜0.85 倍を渡す（自動計算されない。既定は実質無限）。
2. **`scene.ambientColor`** — GI 使用時。ambient エネルギーを載せる（0 だと GI はほぼ no-op）。
   direct light とのエネルギー配分を決め、direct 側を対応して下げてトータル輝度を保つ。
3. **`scene.assumedGamma`** — POV/CueMol 表示と輝度を合わせるなら 2.2 相当（既定 1.0 = 無変換）。
   GI/AO の見た目一致は「まず gamma と ambient/direct のエネルギー配分」で決まる（アルゴリズムより先）。
4. **排他規則** — `strokeEdges` と `objectSpaceEdges` は同時 true で例外。AO と GI は概念的に排他。
   `aaMode=adaptive` と `aoResDiv`（coarse AO）は **`gi` と併用不可**（警告 → フォールバック）。
5. **決定論** — AO/影/GI の乱数は `(pixel, sample)` シードのみで、TBB スレッド数に依存せず
   bit-identical。プリセットを固定すれば再描画は同一結果。

---

## 付録: `RenderOptions` フィールド分類早見表

「設定が多すぎて分からない」を解消するため、全フィールドを **プリセットが触る / client 固有 /
既定固定 / デバッグ** に分類。UI に出す必要があるのは「プリセット」「client 固有」のみ。

| 分類 | フィールド |
|---|---|
| **軸A（プリセット）** | `supersample`, `aaMode`, `aaDepth`, `aaThreshold` |
| **軸B-AO（プリセット）** | `aoSamples`, `aoResDiv`, `aoLowDiscrepancy`, `aoMultiScale`, `aoBentNormal`, `aoFalloffPower`, `aoDiffuseFactor`, `aoIntensity`, `aoMultibounce` |
| **軸B-GI(pt1)（プリセット）** | `gi`, `giIntegrator`, `pt1Spp`, `pt1GatherDiv`, `giBounces`, `pt1Ld`, `pt1Denoise`, `pt1EdgePatch` |
| **軸C（プリセット）** | `shadows`, `shadowSamples`, `lightRadius` |
| **軸D（スタイル）** | `strokeEdges`, `objectSpaceEdges` |
| **client 固有（毎回計算/設定）** | `width`, `height`, `aoDistance`（AO 時）, `scene.ambientColor`（GI 時）, `scene.assumedGamma` |
| **スタイル/任意** | `specularScale`, `pt1SkyMode`, `pt1SkyRadiance`, `giEnvIntensity`, `giIntensity`, `transparency`, `transparentBackground`, `envLights` 系 |
| **既定固定（UI 不要）** | `giMaxDistance`(auto), `pt1Seed`, `maxTransparentLayers`, `pt1HalfRes`, `pt1EdgePatchThresh`, `pt1Upsample*`, `pt1Clamp`, `denoiser`/`denoise*`/`oidn*`, cache 専用（`giSamples`, `giAccuracy`, `giRecordSpacing`, `giGradients`, `giOutlierReject`, `giAdaptive`, `giNormalReject`, `giComponentReject`, `giSeedPerVertex`） |
| **デバッグ（AOV/検証）** | `aaDebug`, `aoResDebug`, `aoWriteAov`, `pt1Stats` |

> **CLI 側の追随（任意の後続作業）**: 現行 `--quality` は `draft/high/ultra` の 3 段だが、段ごとに
> `giBounces` を 1/2/3 と変えており **品質軸と見た目軸が混ざっている**（段を上げると明るさ・
> コントラストも変わる = [§2b](#2b-gipt1-プリセットlow--medium--high--reference) の問題）。
> 修正案: (1) `--quality` から `--gi-bounces` を切り離し、プリセットは `pt1Spp`/解像度だけを変える。
> (2) 命名を本書の `low/medium/high(+reference)` に合わせる。(3) AO 側にも `--ao-quality` 一括
> プリセットを足す。これで CLI と client プリセットが 1:1 対応して保守しやすい（本書では未実施）。

---

## 付録B: 実測コスト（参考）

このリポジトリの分子表面シーン **`data/1ab0_scene6_densurf.pov`**（106,537 三角形 + 1,044 球 +
1,552 シリンダ）を **700×700 / 8 コア（TBB max parallelism 8, macOS）**でレンダリングした
実測値。数値はレンダラ自身の時間（`.pov` の parse/load は除く。client は `Scene` を直接構築する
ため、この値が実コストに相当する）。**AO/基本画質は `render time`、GI(pt1) は gather を含む
`pt1 timing total`**（pt1 の `render time` は primary パスのみを指すため）。

| 軸 | プリセット | 設定 | 実測 |
|---|---|---|---|
| A 基本画質 | ss1 | supersample 1 | 0.03 s |
| A | ss2 | supersample 2 | 0.11 s |
| A | ss3 | supersample 3 | 0.24 s |
| A | ss4 | supersample 4 | 0.38 s |
| A | ss1 + adaptive(depth3) | 縁は 3×3 相当 | 0.12 s |
| B-AO | **low** | ss3, 16spp, out-res | 0.31 s |
| B-AO | **medium** | ss3, 64spp, out-res, multiscale+bent | 0.50 s |
| B-AO | **high** | ss3, 256spp, inline | **47.6 s** |
GI は **`giBounces=1` 固定**の正しい品質ラダー（spp だけを変える）で計測:

| 軸 | プリセット | 設定 | 実測 | foreground 平均輝度 |
|---|---|---|---|---|
| B-GI | **low** | ss1, 8spp / 1-bounce, out-res | 0.64 s | 170.0 |
| B-GI | **medium** | ss1, 32spp / 1-bounce, out-res | 1.32 s | 170.3 |
| B-GI | **high** | ss1, 64spp / 1-bounce, out-res | 2.23 s | 170.6 |
| B-GI | **reference** | ss1, 256spp / full-res / 1-bounce | 7.69 s | 170.5 |

（参考: bounces を段ごとに 1/2/3 と変えた**誤ったラダー**では輝度が 170.0→184.5→189.8 と
上がり、high が明るくなってしまう。品質段では bounces を固定すること。）

読み取れること:

1. **`giBounces` を固定した品質段は輝度が一致（170.0〜170.5）**し、違いはノイズ（spp）だけになる
   ＝正しい品質ラダー。段ごとに `giBounces` を変えると輝度が上がり「別の絵」になる（上表の参考）。
   high と reference が同輝度なのは、spp が明るさを変えず**収束させるだけ**であることの裏付け。
2. **奥行き目的なら GI(pt1) が AO high より大幅に速い**（GI high 2.2 s vs AO high 47.6 s）。
   AO はデノイザが無く `256spp × ss²` の box 平均に頼るため高コスト。GI は OIDN 内蔵で 64spp でも
   クリーンに仕上がる。[§2c](#2c-ao-と-gi-どちらを既定にするか) の「GI 推奨」を実測が裏付ける。
3. **AO medium**（coarse-grid `out-res` + 64spp + low-discrepancy）は 0.5 s と安価。low/medium は
   coarse-grid が効くので速く、high だけ inline 256spp で跳ね上がる（品質段の中で最大の段差）。
4. **pt1 の out-res gather は spp にほぼ線形・ss 非依存**（gather は出力解像度で行う）。ss を上げても
   増えるのは primary パスと silhouette-rim の edge-patch のみ（1-bounce medium: ss1 1.32 s → ss3 で
   +1〜2 s 程度）。一方 **full-res（reference）は ss とともに gather が増える**（ss3 full ≈ ss1 の
   約 9 倍）ので、reference は静止画書き出し専用にとどめる。
5. adaptive AA の高速化は **shading が重いほど効く**。安価な primary のみだと素の grid と同等かやや
   遅い（ss1+adaptive 0.12 s は素の ss1 0.03 s より高いが、同等の縁品質を出す ss3 0.24 s の約半分）。

> 数値は相対比較の目安（scene / 解像度 / コア数 / OIDN の有無で変動）。絶対時間ではなく
> **段間の比**（low→medium→high、AO vs GI）を設計判断に使うこと。
