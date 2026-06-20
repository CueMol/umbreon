# umbreon: Ambient Occlusion + Soft Shadows 実装プラン

ステータス: **計画済み・未着手**（`transp_0619` の bugfix 完了後に実装予定）。本ドキュメントは設計合意の記録。

## Context（なぜこの変更を行うか）

umbreon は CueMol の POV-Ray 出力を Embree 直結で再現する single-bounce レンダラ。
現状は **primary ray + direct local shading のみ**で、AO/影は未実装。
プロジェクトの合意済みターゲットルックは「**direct light + ambient occlusion (+ soft shadows), SINGLE-bounce**」
（radiosity/GI はオーバースペック。PyMOL/ChimeraX/VMD も AO+shadows を使う）。

RTCScene と per-geom GeomKind レコードは整備済みで、`rtcOccluded1` ベースの
AO + soft shadow を追加できる土台がある。アルゴリズムは OSPRay (`~/ext/ospray`,
Apache-2.0) の `scivis`/`ao` レンダラを **C++17 へ移植**（ISPC はコピーせず数式のみ）。

**今回のスコープ（ユーザ決定）**: AO + ハード影 + ソフト影（エリアライト）まで一括、7段階コミット。
**デフォルト挙動（ユーザ決定）**: フラグ未指定時は **OFF**。現行出力と **bit-exact** を維持し、
`--ao-samples` / `--shadows` で opt-in。

> 注: 以下の行番号は調査時点（2026-06-20）の目安。実装時に各ファイルを再読して確定すること
> （探索エージェント間で ±2 行のズレがあったため、構造的アンカー＝関数名/ブロックを優先）。

## 1. アーキテクチャ概要

レンダリングは3層ファサード:
- `umbreon::render()` (`src/umbreon.cpp` ~:71): supersample → fog → boxDownsample → assumedGamma。
  **変更不要**。AO/影は下層で計算され、box平均で自動的にデノイズされる（noise対策の要）。
- `EmbreeRenderer::render()` (`src/render/embree_renderer.cpp` ~:150): Embree scene 構築、
  `std::vector<Light> lights` 変換 (~:410-420)、TBB row loop、front-to-back 透過walk、`shadeHit` ラムダ (~:439)。
- `shadeLocal()` (`embree_renderer.cpp` ~:62): POV ローカル照明カーネル。ambient 初期化 (~:65-67)、per-light loop (~:70-138)。

**接続点:**
- **AO** = ヒット点と静的シーンの性質（ライト非依存）。`shadeHit` 内で1ヒットにつき1回計算し、
  `aoFactor` スカラを `shadeLocal` に渡して **ambient 項のみ**に乗算。`GeomKind::Mesh` ヒットのみ計算、
  プリミティブ（シルエット outline）は `aoFactor = 1.0f`。
- **影** = ライト毎の可視性なので `shadeLocal` の light loop 内。ヒット位置 `P` と `RTCScene` を引数追加し、
  各ライトで `rtcOccluded1`。per-light `shadowFactor ∈ [0,1]` を diffuse+specular 両方に乗算。
- **透過walk との共存**: AO/影レイは `rtcIntersect1` のヒットから飛ばす **secondary ray**。
  front-to-back の `tnear` 前進とは独立で committed `rscene` を読むだけ（walk 不変）。
  透過ジオメトリは **binary occluder** として扱う（OSPRay scivis 同様。2nd walk を避け高速）。
  影/AO は各透過レイヤの色を `Cv`/`Cf` に合成する**前**（= `shadeLocal` 呼び出し位置）に適用。

## 2. 新規ヘルパ関数（`embree_renderer.cpp` 無名 namespace、`faceForward` 直後 ~:34 に挿入）

既存の `Vec3`/`dot`/`cross`/`normalize`/`length`（`scene.hpp:15-33`）を再利用。**行列型は導入しない**（ONB = Vec3×3）。
OSPRay ISPC 数式の純 C++17 移植:

- `struct Onb { Vec3 t, b, n; };  Onb makeFrame(Vec3 N)` — branchless ONB
  （`K = |N.x|>=0.9 ? (0,1,0):(1,0,0); t=normalize(cross(N,K)); b=cross(t,N); n=N`）。
- `Vec3 cosineSampleHemisphere(float u1, float u2, const Onb& f)` — Malley法
  （`phi=2π·u1; cosT=√u2; sinT=√(1−u2)`; local `(cosφ·sinT, sinφ·sinT, cosT)` を ONB 変換）。
- RNG（決定論的、`rand`/`time` 禁止）: `void tea2(uint32_t& v0, uint32_t& v1)`（TEA 8ラウンド）
  ＋ `float u32ToUnorm(uint32_t)`。**シードは (px, py, sampleIndex) のみ**から作る
  （`v0 = px + py*W_hi`, `v1 = sampleIndex`、salt `0x9E3779B9`）→ TBB スレッド数非依存で bit-exact。
  AO 1サンプル毎/影 1サンプル毎に 2 unorm float。**Halton/置換表は移植しない**（低サンプル＋box平均で利得なし）。
- `Vec3 sampleLightDir(const Light& l, float u1, float u2)` — ソフト影用。`l.radius==0` なら `l.L` をそのまま返す。
  それ以外は `makeFrame(l.L)` の周りに角半径 `l.radius` の uniform-disk 摂動（distant光なので方向コーン penumbra）。
- `bool occluded(RTCScene s, Vec3 P, Vec3 dir, float tnear, float tfar)` — `rtcOccluded1` ラッパ
  （`rtcInitOccludedArguments`, 戻り `shadowRay.tfar < 0.0f`）。AO/影 共用。
- `float computeAO(RTCScene s, Vec3 P, Vec3 N, int nSamples, float aoRadius, uint32_t px, uint32_t py, int W_hi)`
  — `nSamples<=0` なら即 `1.0f`。`makeFrame(N)`、cosineSampleHemisphere、`dot(dir,N)<0.01` は hit 扱い、
  さもなくば `occluded(P, dir, eps, aoRadius)`。戻り `1 − hits/nSamples`（cosθ と PDF が相殺するので重み不要）。
- `float computeShadow(RTCScene s, Vec3 P, Vec3 N, const Light& l, int shadowSamples, uint32_t& s0, uint32_t& s1)`
  — `shadowSamples<=1 || l.radius<=0` ならハード影（1本 `occluded`、{0,1}）。
  それ以外は `sampleLightDir` で `shadowSamples` 本平均 → `1 − hits/N`。

## 3. `shadeLocal()` への編集

### 3a. シグネチャ（~:62）
引数追加: `const Vec3& P, float aoFactor, RTCScene rscene, bool shadowsOn, int shadowSamples, uint32_t px, uint32_t py, int W_hi`。

### 3b. ambient 項 — AO 変調（~:65-67）
`mat.ambient * C.* * ambLight.*` の各成分に `aoFactor *` を前置（emission には掛けない、OSPRay `diffuse*ao*aoColor` と同型）。
`aoFactor == 1.0f` のとき `x*1.0f == x`（IEEE no-op）で **byte 一致**。

### 3c. per-light shadow factor（light loop ~:70-138）
backface cull 直後に `shadowFactor = shadowsOn ? computeShadow(...) : 1.0f` を計算し、
ローカル `Vec3 Lc = l.color * shadowFactor` を作って **diffuse 蓄積（~:83-86）と specular/metallic ブロック（~:108, :104-106 の `l.color`）両方で `l.color` を `Lc` に置換**。
`shadowFactor == 1.0f` で `Lc == l.color`（bitwise）→ 影OFF時 bit-exact。

### 3d. mesh-only ゲート（`shadeHit` ~:439-490）
ゲートは**既に構造的**（`rec.kind == GeomKind::Mesh` 分岐 ~:443 vs プリミティブ else ~:458）:
- **Mesh 分岐**: `shadeLocal` 呼び出し前に `aoFactor = (opt.aoSamples>0) ? computeAO(...) : 1.0f`。`P`, `rscene`, `opt.shadows` 等を渡す。
- **プリミティブ分岐（spheres/cylinders = `Material::flatOutline`）**: `aoFactor=1.0f, shadowsOn=false, shadowSamples=0` を**リテラルで**渡す。
  これが **flatOutline ゲート**（AO/影をシルエット outline に絶対適用しない。必須）。
- `shadeHit` ラムダの `org` キャプチャ問題に注意（`org` は loop 内 ~:516 でラムダ定義 ~:439 より後に定義）。
  → `shadeHit` シグネチャに `const Vec3& org, uint32_t px, uint32_t py` を追加し、唯一の呼び出し箇所（~:579）を更新。
  内部で `P = org + rd * rh.ray.tfar` を計算。`rscene`/`opt`/`lights`/`records` は `[&]` で既にキャプチャ済み。

## 4. RenderOptions + CLI フラグ

`render_types.hpp` に既存: `aoSamples`（~:18）, `aoDistance`（~:19）, `shadows`（~:25）— **いずれも現状レンダラ未参照**。再利用＋3フィールド追加。

**`render_types.hpp`（~:19 以降に追加）:**
```cpp
float aoIntensity = 1.0f;   // AO strength: aoFactor = 1 - aoIntensity*(1-rawAO)
int   shadowSamples = 1;    // shadow rays per light (>1 = soft)
float lightRadius = 0.0f;   // angular radius (deg) for soft shadows; 0 = hard
```
`aoDistance` → AO レイ `tfar`（OSPRay `aoRadius`）。`shadows` → 影マスタトグル。`aoSamples` → hemisphere レイ数。

**bit-exact のためのデフォルト反転（決定済み）:**
- `RenderOptions::aoSamples` を `16 → 0`（~:18）
- `RenderOptions::shadows` を `true → false`（~:25）
- `Options`（`cli.hpp` ~:20 以降）: `int aoSamples = 0; bool shadows = false; int shadowSamples = 1; float lightRadius = 0.0f; float aoIntensity = 1.0f;`

**`cli.cpp` フラグ追加（live な `--ao-distance` ~:57 のパターンに倣い ~:58 以降）:**
`--ao-samples`（int）, `--ao-intensity`（float）, `--shadows on|off`（`parseBool` 再利用 `cli.cpp:9`）,
`--shadow-samples`（int）, `--light-radius`（float, deg）。`printUsage`（~:172 以降）に対応行追加。

**`main.cpp` 配線**: `.inc` ブロック（~:240-247、既存 `ropt.aoDistance` 隣）と `.pov` ブロック（~:204-209）両方に
`ropt.aoSamples/aoIntensity/shadows/shadowSamples/lightRadius = opt.*;` を追加。
ライト変換（~:410-420）で `l.radius = radians(opt.lightRadius);`。

## 5. ライトモデル（ソフト影）

内部 `Light` 構造体（`embree_renderer.cpp` ~:38-42）に **1フィールドのみ追加**:
```cpp
float radius = 0.0f;  // angular radius (radians) for soft shadows; 0 = hard
```
`scene.hpp` の `DistantLight` は**触らない**（per-light radius は今回不要。グローバル `--light-radius` で十分）。

- `shadowSamples<=1 || l.radius<=0` → ハード影（1本 `rtcOccluded1`、デフォルト経路）。
- それ以外 → ソフト: `sampleLightDir`（`makeFrame(l.L)` 周りの uniform-disk を `tan(l.radius)`≈`r·l.radius` でスケール、再正規化）で
  `shadowSamples` 本平均。distant光なので距離項なしの方向コーン penumbra（"area-light Monte Carlo" 意図に合致）。

## 6. 決定論 / bit-exact 戦略

現行の locked 回帰テスト（`tests/test_render.cpp`）は `RenderOptions` を直接構築し `aoSamples`/`shadows` を**設定しない**。
保証:
1. **AO early-out**: `opt.aoSamples > 0` のときのみ `computeAO`、さもなくば `1.0f`。`x*1.0f==x`。
   ただし `RenderOptions::aoSamples` の現行デフォルトは **16** → §4 でデフォルトを **0** に変更（最重要）。
2. **影 early-out**: `shadowsOn = opt.shadows`（デフォルト **false** に変更）。`false` で `shadowFactor=1.0f` リテラル、`Lc==l.color`。
3. **プリミティブゲート**: outline は常に `aoFactor=1.0f, shadowsOn=false`（C系/seam テスト不変）。
4. **演算順序不変**: AO/影は既存アキュムレータへの乗算のみ。no-AO 経路の float 演算順を再構成しない。
5. **TBB 決定論**: RNG シードは `(px, py, sampleIndex)` のみ。スレッド数/grain 非依存。

→ `aoSamples=0` + `shadows=false`（新デフォルト）で全経路が `1.0f` 乗算に退化し **byte 完全一致**。
**コミット2（デフォルト反転直後）で `ctest` green を確認**してから実アルゴリズムに進む。

## 7. テスト計画

`tests/test_render.cpp` 末尾（`return s.report();` ~:659 直前）に追加。**合成シーン**を使用
（POV `.inc` ライトは `no_shadow` 確認済み → POV ground-truth は CueMol 再export が必要。別タスク `data/run_ao_ref.sh` として非ブロッキングで後回し）。
既存イディオム（5×5 ortho、center-pixel assert、`makeOrthoCam`/`makeQuad`/`makeKeyLight`/`approx`）に倣う:

- **AO-OFF identity（bit-exact guard）**: 既存 mesh shading シーンを `aoSamples=0` でレンダし locked 値 `0.30/0.36/0.42` を assert。
  さらに開いた quad（上方遮蔽なし）を `aoSamples=8` でレンダし center 不変（開半球 → AO≈1、ノイズ許容 ~1e-2）。
- **AO darkening under a ceiling**: floor quad + 直上に近接 ceiling quad。`ambient>0`、center の ambient チャネルが no-AO より厳密に小、direct diffuse は不変。`aoSamples=64`、不等式 assert。
- **Hard shadow**: 斜めキーライト + 遮蔽三角形。`shadows=true, shadowSamples=1` で center diffuse が ambient-only まで低下、`shadows=false` で点灯値と一致。
- **Soft-shadow penumbra（許容緩め）**: `lightRadius>0, shadowSamples=16`。penumbra ピクセルが全点灯と全影の**中間**（3ピクセル単調性 assert 推奨、最もflaky）。
- **flatOutline gate**: `Material::flatOutline()` 球を `aoSamples=64, shadows=true` でレンダし center が no-AO/no-shadow 値と**厳密一致**。

CMake/ctest 配線は変更不要（`test_render` 既登録）。実行は **`task test`**（`Taskfile.yml:98` → `ctest --test-dir build --output-on-failure`）。

## 8. コミット順（小さく独立検証可能、フルスコープ）

1. **ヘルパのみ（挙動不変）**: `makeFrame`/`cosineSampleHemisphere`/`tea2`/`u32ToUnorm`/`occluded`/`sampleLightDir`/`computeAO`/`computeShadow` を ~:34 に追加。未使用 → `ctest` green（コンパイル証明）。
2. **bit-exact デフォルト反転**: `RenderOptions::aoSamples=0`（~:18）, `shadows=false`（~:25）。既存 `ctest` green（ベースライン保持の安全網）。
3. **AO 計算 + ambient 変調**: `shadeLocal` に `aoFactor` 追加、mesh 分岐で `opt.aoSamples>0` ゲート、プリミティブは `1.0f`。→ AO-OFF identity + under-ceiling テスト。
4. **AO CLI + 配線**: `--ao-samples`/`--ao-intensity`、`main.cpp` 両ブロック配線、`--ao-distance`→`aoDistance` 再利用。
5. **ハード影**: `shadeLocal` に `P/rscene/shadowsOn`、per-light `rtcOccluded1`、`Lc` 折り込み、`shadeHit` で `P`+トグル渡し（プリミティブ `shadowsOn=false`）。`--shadows on|off` + 配線。→ hard-shadow + flatOutline-gate テスト。
6. **ソフト影（エリアライト）**: `Light::radius`/`shadowSamples`/`--shadow-samples`/`--light-radius`、`computeShadow` ソフト分岐。→ penumbra テスト。
7. **テスト固め + 任意 POV ref**: 新 assert ロック、任意 `data/run_ao_ref.sh`（shadow-casting 再export POV シーン、非ブロッキング）、`printUsage`/docs 更新。

各コミットは独立して green。コミット2 が bit-exact チェックポイント。

## 9. リスク / 注意点

- **MC ノイズ**: 低サンプルは粒状。supersample box平均（`umbreon.cpp` ~:90）が `ss²` 個の jitter 済み AO を自動平均。テストは不等式/許容範囲で。視覚用途は AO≥16, 影≥8、テストは 64。
- **self-occlusion / shadow acne**: AO/影レイ原点に法線方向 eps オフセット（`eps = max(length(P)*1e-5f, 1e-4f)`、walk の tnear 前進スケール ~:612 と整合）。backface 時 `P -= 2·eps·Ng`。`RTC_SCENE_FLAG_ROBUST`（既設 ~:155）も誤自己交差を低減。
- **supersample 相互作用**: RNG は **hi-res** 座標/幅（`W_hi`）を使用 → 各 subpixel が別シード → 正しい jitter。`--supersample` × `--ao-samples` でレイ数が乗算される点を docs 明記。
- **透過相互作用**: occlusion レイは透過を不透明扱い（binary）。ガラス背後で AO/影が過剰に暗くなり得るが OSPRay scivis 同等＋高速。コメント明記。
- **flatOutline ゲートは必須**: プリミティブ分岐に AO/影が漏れるとシルエットが誤って暗くなる。`rec.kind==Mesh` ゲートが唯一の防御点。
- **`aoIntensity` 意味**: `aoFactor = 1 − aoIntensity·(1 − rawAO)`（intensity=1 で full AO、0 で無効化の clean no-op）。
- **デフォルト反転の波及**: `RenderOptions::shadows` true→false は現状レンダラ未参照ゆえ安全だが、`src/` 内の他参照を grep 確認（render_types 外に無いことを確認済み）。

## 検証（end-to-end）

1. **bit-exact**: コミット2 後と各コミット後に `task test`（= `ctest --test-dir build --output-on-failure`）→ 既存 + 新規 green。
2. **視覚確認**: `./build/umbreon_cli <scene>.pov --ao-samples 16 --ao-distance <r> --shadows on --light-radius 2 --shadow-samples 16` でレンダし PNG 目視。
3. **回帰**: フラグ無し `./build/umbreon_cli <scene>.pov` 出力が現行 `docs/match` と byte 一致（PNG-vs-PNG、外部 Python/skimage。tool 内 `--compare` は PPM のみ）。
4. **任意 ground-truth**: shadow-casting 再export POV シーンを作れば `data/run_ao_ref.sh` で PSNR/SSIM 比較（非ブロッキング）。

## 主要編集ファイル
- `src/render/embree_renderer.cpp`（ヘルパ ~:34 / `shadeLocal` ~:62 / `shadeHit` ~:439 / `Light` ~:38 / ライト変換 ~:410）
- `src/render/render_types.hpp`（`RenderOptions`: aoSamples→0 ~:18, shadows→false ~:25, 新3フィールド）
- `src/cli.cpp`（フラグ ~:57+ / usage ~:172+） / `src/cli.hpp`（`Options` ~:20+）
- `src/main.cpp`（配線 ~:204-209 pov / ~:240-247 inc）
- `tests/test_render.cpp`（~:659 直前に AO/影テスト追加）
