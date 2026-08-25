# Tone Hatching / スクリーントーン NPR モード（--hatch）実装プラン

*Plan location:* `docs/plans/npr-tone-hatching.md`
*Status:* Phase 1-3 実装済み（`feat/npr-tone-hatching`）: Line/Dot 全マーク・摂動・
プリセット 6 種・`--hatch-layer`・セクション別スタイル（`hatchGroup` AOV +
`Scene::groupHatchStyle` + `--hatch-style ID=spec`）・base/ink 4 通り・
`--hatch-tone` / `--hatch-min-contrast`・`aoResFallbackSppMul`。
実装時の逸脱: `aoResFallbackSppMul` のライブラリ既定は **1**（指示書案の 4 だと既存の
coarse AO 出力が変わり `test_ao_coarse` の許容を破るため byte-identical を優先）。
4 は CLI の NPR 既定（`--hatch` 時、明示指定なし）として適用する。
*Brief:* `docs/plans/npr-tone-hatching-INSTRUCTIONS.md`（外部設計議論の合意事項）
*Superseded in part:* Dot の 50% 反転（§6.2 F2）は被覆→半径テーブル方式に、stipple プリセットは
`LayerKind::Stipple` に、`widthPx <= 2·step` クランプは `<= spacing` に置き換えた。
`docs/plans/npr-hatch-mark-geometry.md` を参照。

> 検証ノート: 本文書の `file:line` はすべて作業ツリー（`main` @ `ce89a25`、クリーン）に対して
> 実際に検証済み。指示書との食い違いは §2 に **Discrepancy:** として明示した。
> 指示書 §2.3 の擬似コード自体にも数値解析上の欠陥が 4 件見つかったため（§6.1 F1/F2/F4/F5）、
> 本プランでは修正版の式を正とする。

---

## 1. Summary

`--hatch` で有効になる NPR シェーディングモードを追加する。陰影を連続階調ではなく
線の密度（ハッチング／クロスハッチ／網点・スティップル）で表現する。
Praun et al. *Real-Time Hatching* (SIGGRAPH 2001) の Tonal Art Map を、テクスチャを持たない
手続き的生成（線インデックスのビット演算で出現レベルを決める）として実装する。
フェード導入は Webb et al. (NPAR 2002) の手続き版。

パイプライン上の位置は「トーンの**生成**は `hit_shader` 内（AO と同じ場所）、インクの**消費**は
`renderFrame()` 最終段の `applyAssumedGamma` の**後**」。ストロークエッジ（hi-res 合成 →
downsample）とは意図的に逆で、トーンを hi-res で作り box-downsample でアンチエイリアスしてから
最終解像度で二値化する。これにより supersample を上げるほどトーンが滑らかになり、
線幅は出力ピクセル基準で一定に保たれる。

既定（`hatch.enable = false`）ではバッファ確保・書き込み・後処理のすべてが完全にゲートされ、
既存出力は **byte-identical**。既存の `--edges`（stroke edges）/ `--obj-edges` とは独立に併用可能で、
ハッチが階調を、エッジが輪郭を担当する。

---

## 2. 現状の検証済み所見（file:line 付き）

- **`renderFrame()` の全体フロー** — `src/umbreon/render/pipeline.cpp:20-264`。
  正規化ブロック `:51-79`（`aaMode==1 && gi` フォールバック `:60-65`、`aoResDiv>1 && gi`
  フォールバック `:74-79`）→ `setPhasePlan` `:88-90` → `render()` `:117` → fog `:126-129`（hi-res）→
  strokeEdges `:131-173`（hi-res、downsample 前）→ downsample `:185-231` → denoise `:233-260` →
  `applyAssumedGamma` `:262`。**gamma の後には何も無い** — そこが `applyHatch` の挿入点。
- **AOV の確保は一元化** — `allocateFrameBuffers()` = `src/umbreon/render/embree_renderer.cpp:471-532`。
  書き込みは `storeShadingChannels()` `:166-238` / `storeGBufChannels()` `:240-261`。
  AO 系 AOV（contactAo 等）は `aoWriteAov` ゲートで shading 側に書かれ、downsample 対象
  （`pipeline.cpp:201-210`、1ch）。新規 AOV は自前の `!empty()` ゲートが必要（`pipeline.cpp:213-216` の規約）。
- **adaptive AA (aaMode==1) との整合** — replicated サブピクセルも `storeShadingChannels` を通る
  （`embree_renderer.cpp:1656-1699`）ため、トーン AOV を contactAo と同じ経路（shading channel）に
  乗せれば追加のフォールバック正規化は不要。`probeGBuffer`（`transparency.hpp:196-304`）は
  shading-free なので、トーンを G-buffer 側に置いてはならない。
- **トーン生成に必要な量の所在** — `shadeLocal()` の per-light ループは
  `src/umbreon/shading/shading.hpp:96-179`。`ndl` `:102`、`shadowFactor` `:109-111`、
  brilliance 適用後の `d` `:117-123`。**拡散スカラー和はどこにも実体化されていない**
  （`dk*C*Lc` を直接 `out` に畳み込む `:124-127`）→ optional の積算ポインタを追加する必要がある。
  principled 側は `src/umbreon/shading/principled.hpp:139-269`（hard shadow 枝 `:184-222`、
  area light 枝 `:223-269` — 後者は可視性がサンプルループ内 `:238` にあり shadowFactor は分離不能）。
- **AO の contact/shape** — `aoShadeForHit()`（`src/umbreon/shading/hit_shader.hpp:124-142`）の
  戻り値 `AoShade`（`src/umbreon/ao/ao_shade.hpp:30-35`）に `aov.contact/shape` が入るのは
  quality パス（`opt.aoEnhanced() || opt.aoWriteAov`、`ao_shade.hpp:49-69`）のみ。
  legacy パス（`computeAO`）は openness のみ。`aoFactor`/`diffuseAo` はいずれも openness 由来
  （`ao_shade.hpp:99-115`）。**Discrepancy:** 指示書 §2.5 は「`aoWriteAov` が
  `hs.contactAo` を書いている箇所が起点」とするが、AOV 書き出しに依存すると
  primitive（sphere/cylinder）枝で contact/shape が得られない（`hit_shader.hpp:207-220` は
  mesh 枝のみ、primitive 枝は AO AOV を一切書かない）。本プランでは AOV でなく
  **shader 内の `ao.aov` を直接使う**（mesh 枝 `:198-203` / primitive 枝 `:346-350` の両方で取得可能）。
- **Discrepancy: `frame.albedo` は pigment 色 `C` のみ** — `hs.albedo = C`
  （`hit_shader.hpp:178, 322`）。指示書 §2.6 の「`mat.diffuse * pigment`」は別フィールド
  `hs.giReflectance`（`hit_shader.hpp:237-243`、GI 限定）。ベタ塗り下地には陰影前のフラットな
  pigment 色こそが正しいので、`frame.albedo` をそのまま使う（挙動は指示書の意図通り、記述だけ訂正）。
  なお albedo バッファは `aoWriteAov || (gi && denoiser && demodulate)` のときのみ確保
  （`embree_renderer.cpp:504-507`）→ hatch 用の確保ゲートを追加する。
- **`surfAlpha` は流用不可** — strokeEdges ゲートで確保・書き込み（`embree_renderer.cpp:485-488,
  251-254`）、downsample されず、意味も「第一ヒットの不透明度」でヒット被覆率ではない。
  **独立した `hatchMask` を持つ**（指示書 §2.2 の判断を追認）。
- **透過の第一ヒット規約** — `integratePixel()` は `nearDepth == 0.0f` の枝
  （`src/umbreon/shading/transparency.hpp:394-417`）で全 AOV を最近接ヒットから捕獲する。
  hatchTone / hatchMask も同じ規約（第一ヒットのみ、背後レイヤは捨てる）に従う。
- **per-section スタイルの既存パターン** — `Scene::groupEdgeStyle`（`src/umbreon/scene.hpp:467-473`）、
  2 段フォールバックの `resolveStrokeStyle()`（`src/umbreon/edges/stroke_render.cpp:1169-1195`：
  空テーブル→グローバル、範囲外 group→`se.defaultStyle`）、CLI `--edge ID=spec`
  （`src/bench/cli.cpp:790-810`：先頭 `=` 分割、`_show` 前置除去）、名前→index 解決
  （`src/bench/scene_setup.cpp:358-375`、`--list-groups` は `:73-92`）。`groupHatchStyle` は
  これと完全対称に作れる。`hs.group` は両プリミティブ枝で無条件に設定される
  （`hit_shader.hpp:277, 457-459`）。
- **public API の 3 点同期** — `CMakeLists.txt:216-245`（install(FILES) 9 ヘッダ）、
  `docs/api/libumbreon.md` の表、ヘッダ内マーカー。新規公開ヘッダは 3 箇所すべてに追加する。
  NPR エッジの先例: 「オプション型のみ公開、実装ヘッダは internal」（`CMakeLists.txt:240-243`）。
  本機能は指示書 §2.13 の要請により `applyHatch` 関数も公開する（例外である旨を記録）。
- **Discrepancy: `smoothstep` ヘルパー・ノイズ関数・汎用ハッシュは存在しない** —
  smoothstep は `stroke_render.cpp:766, 830` にインライン展開があるのみ。value/gradient noise、
  PCG/Wang hash は非 experimental コードに皆無（`hashU32` は
  `experimental/irradiance_cache/irradiance_cache.hpp:181-188`）。→ `npr/` 内にローカル実装（§6.4）。
- **Discrepancy: `--quality` は draft/high/ultra**（指示書 §3.10 が引く低中高ではない）で、
  すべて `gi=true` を焼き込む（`src/bench/cli.cpp:497-532`）。hatch Ink モードは GI を切るため
  `--quality` に NPR 段を足すのは筋が悪い。**追加しない**（§7 参照、AO 側の既定で対応）。
- **Discrepancy: `denoiseAtrous` のガイドは `frame.normal` + `frame.position`** で、
  `position` は GI 時のみ確保（`experimental/irradiance_cache/denoise_atrous.cpp:35-40`、
  `embree_renderer.cpp:521-524`）。指示書 §2.10 の `toneSmooth`（SVGF ラッパ）は
  ガイドの前提が崩れるため**スコープ外に格下げ**（§10）。優先順位の高い
  `aoResDiv=-1` + `aoLowDiscrepancy` + フォールバック追加サンプリングで解く。
- **`task lint` は存在しない** — Taskfile.yml のタスクは build/test/render/deps/clean と
  `:static` 系のみ。検証は `task test`（`Taskfile.yml:182-187` = configure:static + build + ctest）。
- **テスト規約** — 自前ハーネス `tests/test_util.hpp`（Suite/check/report）。byte-identical は
  `framesEqual`（`tests/test_ao_coarse.cpp:76-81`）、スレッド決定性は `tbb::global_control` の
  scoped block（`:302-319`）、AOV bitwise 比較は `tests/test_adaptive_aa.cpp:552-556`。
  登録は `tests/CMakeLists.txt` の 3 行パターン（`:93-95`）。
- **`data/1ab0_scene4_toon1.pov` は存在する**が、参照する `.inc` が Git-LFS
  （`tests/CMakeLists.txt:5-10`）→ 回帰テストには合成シーンを使い、toon1 は目視確認専用にする。
- **group-alpha マルチパス**（`src/umbreon/umbreon.cpp:126-249`）は各パスで full pipeline を回し
  表示エンコード済みの値を重み付き合成する。`applyHatch` は各パス内で実行され、格子は
  スクリーン固定なのでパス間で線位置は一致する。半透明セクションはインクも重みで
  ブレンドされる — CueMol blendpng と同じ振る舞いであり許容（§4.4）。

---

## 3. データ構造の変更

### 3.1 FrameResult 追加 AOV

| AOV | 型 | 内容 | 確保条件 | downsample |
|---|---|---|---|---|
| `hatchTone` | `vector<float>` w·h | 陰影トーン（連続値・リニア、1=紙白側） | `opt.hatch.enable` | yes, 1ch |
| `hatchMask` | `vector<float>` w·h | 第一ヒット有無（1=サーフェス、0=背景） | `opt.hatch.enable` | yes, 1ch |
| `hatchGroup` | `vector<uint16_t>` w·h | 群 ID（hi-res のまま保持） | Phase 3、per-section 有効時 | **no**（整数） |
| `hatchUv` | `vector<float>` w·h·2 | 物体空間 UV（将来拡張） | **今回は実装しない** | （設計のみ） |

`frame_result.hpp` の AO ブロック（`:75-78`）の後に hatch ブロックとして追加。
確保は `allocateFrameBuffers()`（`embree_renderer.cpp:471-532`）に
`if (opt.hatch.enable) { res.hatchTone.assign(npix, 0.f); res.hatchMask.assign(npix, 0.f); }` を追加。
`hatchUv` はフィールドすら足さず、`applyHatch` 側を「UV 引数が null ならスクリーン座標」という
分岐構造にしておくことで拡張点だけ確保する（指示書 §2.12）。

### 3.2 新規公開ヘッダ `render/hatch_types.hpp`

`edge_types.hpp` と同格の公開型ヘッダ。全型を Phase 1 で確定させる（Phase 2-3 は解釈側の実装のみ）。

```cpp
enum class HatchMode : uint8_t { Ink = 0, Over = 1 };
enum class HatchBase : uint8_t { Paper = 0, Albedo = 1 };
enum class HatchInk  : uint8_t { Fixed = 0, FromAlbedo = 1 };
enum class LayerKind : uint8_t { Line = 0, Dot = 1 };

struct MarkStyle {
  float edgeSoftness  = 0.5f;   // AA half-width in FINAL px (>=0.5 enforced)
  float toothAmp      = 0.0f;
  float toothScalePx  = 3.0f;
  unsigned seed       = 0;
  // Dot
  float shapeExponent = 2.0f;   // Lp: 1=diamond 2=circle >=16=square
  float dotAspect     = 1.0f;
  float dotAngleDeg   = 0.0f;
  float jitter        = 0.0f;   // 0=halftone .. 0.4=stipple (clamped to 0.5)
  bool  invertAbove50 = true;
  // Line
  float wobbleAmpPx   = 0.0f;
  float wobbleWavePx  = 40.0f;
  float widthJitter   = 0.0f;
  float strokeLenPx   = 0.0f;   // 0 = infinite
  float strokeGapPx   = 0.0f;
  float strokeTaper   = 0.3f;
};

struct HatchLayer {
  LayerKind kind   = LayerKind::Line;
  float angleDeg   = 45.0f;
  float spacingPx  = 10.0f;     // base spacing S in FINAL px
  int   subdiv     = 2;         // K
  float widthPx    = 1.1f;      // full line width in FINAL px
  float toneHi     = 0.95f;     // level-0 threshold (display-encoded tone)
  float toneLo     = 0.55f;     // level-K threshold
  float fadeInv    = 16.0f;
  float opacity    = 1.0f;
  MarkStyle mark;
};

struct ToneRecipe {
  float diffuseWeight = 1.0f;
  float ambient       = 0.12f;
  float contactAoPow  = 1.0f;
  float shapeAoPow    = 0.6f;
  float blackPoint    = 0.0f;
  float whitePoint    = 1.0f;
  float gamma         = 1.0f;   // linear-domain artistic curve
  float specularCut   = 0.0f;   // >0: blow out to paper where specular exceeds
};

struct GroupHatchStyle {        // Phase 3
  bool  enable = true;
  HatchBase base = HatchBase::Paper;
  HatchInk  ink  = HatchInk::Fixed;
  float inkColor[3] = {0.0f, 0.0f, 0.0f};
  int   layerMask = 0x7;
  float toneScale = 1.0f;
};

struct HatchOptions {
  bool enable = false;          // MASTER gate: false => byte-identical default path
  HatchMode mode = HatchMode::Ink;
  HatchBase base = HatchBase::Paper;
  HatchInk  ink  = HatchInk::Fixed;
  float inkColor[3]   = {0.0f, 0.0f, 0.0f};   // DISPLAY-encoded (post-gamma domain)
  float paperColor[3] = {1.0f, 1.0f, 1.0f};   // DISPLAY-encoded
  float inkMinContrast = 0.25f;
  int   toneLevels = 0;         // 0 = continuous
  int   albedoQuantize = 0;     // 0 = off; N = round(a*N)/N on the base color
  bool  transparentBackground = false;  // copied from RenderOptions by the pipeline
  ToneRecipe tone;
  std::vector<HatchLayer> layers;       // empty => pen-cross preset at normalization
};
```

- `RenderOptions` 末尾（`objectSpaceEdges` `render_options.hpp:330` の後）に
  `// --- tone hatching (--hatch) --- defaulted OFF` バナー + `HatchOptions hatch;`。
- `Scene` に `std::vector<GroupHatchStyle> groupHatchStyle;`（`groupEdgeStyle` `scene.hpp:473` の直後、
  Phase 3 で消費、型は Phase 1 で確定）。
- **色の規約**: `inkColor`/`paperColor` は表示（display-encoded）空間の値。`applyHatch` は
  `applyAssumedGamma` 後に走るため、リニアで持つ既存の `EdgeClassStyle::color` とは規約が異なる
  （ヘッダコメントに明記する）。純黒 `{0,0,0}` は両規約で不変。

### 3.3 公開 API とビルド

- `src/umbreon/npr/hatch_shade.hpp` — **公開**（指示書 §2.13。レンダラ非依存の image op として
  CueMol 側から合成トーンを流し込める・ユニットテストが合成トーンだけで書ける、が根拠）:

```cpp
// npr/hatch_shade.hpp  (PUBLIC)
void applyHatch(int w, int h, float* rgba,          // display-encoded RGBA, in place
                const float* tone, const float* mask,
                const float* albedo,                // nullable (base=Albedo / ink=FromAlbedo)
                const HatchOptions& opt);
```

- `src/umbreon/npr/hatch_shade.cpp` — 実装。`CMakeLists.txt` の
  `add_library(umbreon STATIC ...)`（`:118-142`）にソースを追加。
- `src/umbreon/npr/hatch_ink.hpp` — **internal**。格子・マーク形状・摂動・hash/noise の
  純関数群（テストから直接 include して検証する。`tests/test_ao_coarse.cpp:13` が internal ヘッダを
  include する先例）。
- install(FILES) は 9 → **11** ヘッダ（`render/hatch_types.hpp` → `include/umbreon/render`、
  `npr/hatch_shade.hpp` → `include/umbreon/npr`）。`CMakeLists.txt:216-222` の SSOT コメント、
  `docs/api/libumbreon.md` の表、ヘッダ内マーカーの 3 点を同期更新。
  `render_types.hpp`（4 行アンブレラ、`render_types.hpp:12-15`）に `hatch_types.hpp` を追加。

---

## 4. パイプライン改修

### 4.1 正規化ブロック（挿入点: `pipeline.cpp:79` 直後、`setPhasePlan` `:88-90` の前）

既存 2 つのフォールバック（`:60-65`, `:74-79`）と同じ形・同じ場所。cost model が正規化後の
オプションを見るため、ここより後ろでは進捗バーが壊れる（指示書 §2.9 の要請どおり）。

```cpp
if (hi.hatch.enable) {
  if (hi.gi && hi.hatch.mode == HatchMode::Ink) {
    umbreon::logMessage(umbreon::LogLevel::Warning,
                        "--hatch ink does not use GI; disabling --gi");
    hi.gi = false;
  }
  if (hi.hatch.mode == HatchMode::Ink) {
    hi.denoiser = static_cast<int>(DenoiserBackend::None);
    hi.pt1Denoise = false;
  }
  if (hi.hatch.layers.empty()) hi.hatch.layers = hatchPresetLayers(HatchPreset::PenCross);
  normalizeHatchLayers(hi.hatch);   // K_eff clamp + perturbation clamps (see 6.3)
  hi.hatch.transparentBackground = hi.transparentBackground;
}
```

`Over` モードでは明示指定の `gi` を尊重する（`data/1ab0_scene4_toon1.pov` のような toon 系では
色面が最終画に残るため）。既定はどちらのモードでも `gi=false`（`render_options.hpp:93`）。

色デノイザを NPR で通さない理由（指示書 §2.9 の要請により明記）:
1. `denoiseAtrous` は albedo 分離後の照度平滑を前提とし（`denoise_atrous.cpp:63-75`）、
   ハッチトーンは独立 AOV なので色に掛けても二値化前のトーンに届かない。
2. OIDN は低周波照度向けの学習ベースで `denoiseSigmaL` 相当の明示制御もなく、
   二値化直前のトーンには挙動が読めない。

### 4.2 downsample リスト（`pipeline.cpp:228` の giOcclusion の後）

```cpp
if (!frame.hatchTone.empty())
  frame.hatchTone = boxDownsample(frame.hatchTone, frame.width, frame.height, 1, ss);
if (!frame.hatchMask.empty())
  frame.hatchMask = boxDownsample(frame.hatchMask, frame.width, frame.height, 1, ss);
```

個別 `!empty()` ゲート（`pipeline.cpp:213-216` の規約に従う）。`hatchMask` の box 平均が
シルエット境界の被覆率 AA をそのまま与える。cost model
（`progress_cost_model.hpp:266-268`）に `if (hi.hatch.enable) chans += 2;` を追加。

### 4.2.5 Ink モードの下地塗り（Phase 1 実装時の設計修正）

当初設計では `applyHatch` が最終段でサーフェス画素を「下地×インク」で**置き換え**ていたが、
これでは downsample 前に hi-res 合成済みのストロークエッジのうち、サーフェス上に乗る
**内部 silhouette 線がすべて消える**（背景側に描かれる外周輪郭だけが残り、実質 Outline モードに
見える）ことが Phase 1 の実機確認で判明した。修正: `--edges-only` のブランク処理
（`pipeline.cpp:148-160`）と同じパターンで、**Ink モードの下地（紙白／albedo ベタ、
`albedoQuantize` 込み）を fog・エッジパスの前に hi-res・リニア空間で塗り込む**。paperColor は
表示値なので `pow(d, 1/assumedGamma)` でリニア化して塗る（gamma で往復）。最終段の
`applyHatch` は両モードとも**インクの乗算のみ**（`px *= (1-m) + m·f`）となり、エッジインクは
構造的に生き残る。副次効果として「ベタ塗り下地は無陰影」（陰影はハッチ密度のみが担う）も
この段階で構造的に保証される。回帰テスト: `tests/test_hatch.cpp` の
「edges survive ink」（深度ギャップ線がハッチ合成後も残ることを、トーンを紙白に固定して検証）。

### 4.2.6 インク解像度（実運用からの設計改訂: `--hatch-res hi` を既定に）

当初設計は「インクは最終解像度で 1 回だけ二値化」（§2.1 の順序）だったが、実運用で
「線を 1 本ずつ解像する必要はなく、線描らしいテクスチャが出れば良い。出力解像度による
最小ピッチ 2px の制限の方が問題」という結論に至った。改訂: **既定を supersample 解像度での
インク合成（`HatchOptions::inkHiRes = true`, `--hatch-res hi`）とする**。gamma を hi-res で
先に適用してインクを表示空間で合成し、box downsample が表示空間でストロークを平均する
（サブピクセル線は正確な被覆トーンの細粒に溶ける）。**px 単位のレイヤパラメータは
出力ピクセル単位のまま**で、`renderFrame` が hi-res 格子へ変換（×ss。edgeSoftness は
デバイスピクセル量なので変換しない）するため、同じ指定値で ss を変えても見た目は不変、
実効ピッチ下限は 2/ss 出力 px になる。`--hatch-res out` で従来のピクセル精度ストローク
（下限 2px）も選択可。hi モードでは表示エンコード後になるためカラーデノイザは正規化オフ、
hatch AOV はエッジ G-buffer と同様 hi-res のまま保持される。

### 4.3 `applyHatch` の呼び出し（`pipeline.cpp:262` の `applyAssumedGamma` の直後）

```cpp
applyAssumedGamma(frame, scene.assumedGamma);
if (hi.hatch.enable && !frame.hatchTone.empty()) {
  applyHatch(frame.width, frame.height, frame.color.data(),
             frame.hatchTone.data(), frame.hatchMask.data(),
             frame.albedo.empty() ? nullptr : frame.albedo.data(), hi.hatch);
}
return frame;
```

表示空間で合成する根拠は指示書 §2.1（色インクを表示色で指定するため。
`image_ops.hpp:28-32` の group-alpha 合成と同じ判断）。

### 4.4 group-alpha マルチパスとの関係

`renderImpl`（`umbreon.cpp:141-249`）は各パスで `renderFrame` を full pipeline で回すため、
ハッチも各パスで掛かり、`srgbEncodeF` 済みの値が重み合成される（`umbreon.cpp:220-224`）。
格子はスクリーン座標の純関数なのでパス間で線位置は厳密に一致し、半透明セクションの
インクは α で薄まる。これは CueMol blendpng が完成 PNG を合成するのと同じ意味論であり許容。

### 4.5 fog

`applyFog` は hi-res で `frame.color` にのみ掛かり（`pipeline.cpp:126-129`、`fog.cpp:17-36`）、
`hatchTone` は素通り。「色は霞むが線の密度は手前と同じ」になるが、現行 silhouette 線が
fog で色だけ薄まる挙動（`stroke_render.cpp:723-745`）と整合するため**これを仕様とする**
（指示書 §2.11）。fog 自体をトーンに畳む案は将来課題（silhouette 側の変更を伴うためスコープ外）。

---

## 5. シェーディング改修（トーン生成）

### 5.1 ToneAccum タップ（`shadeLocal` / `shadePrincipled`）

拡散スカラー和は実体化されていない（§2）ため、両シェーダに optional 引数を足す。
既定 `nullptr` で挙動・コード生成とも不変（byte-identical）。

```cpp
struct ToneAccum { float diffuse = 0.0f; float specular = 0.0f; };
// shading.hpp shadeLocal(..., ToneAccum* toneAcc = nullptr)
// principled.hpp shadePrincipled(..., ToneAccum* toneAcc = nullptr)
```

- `shadeLocal` の per-light ループ（`shading.hpp:124` 付近）:
  `if (toneAcc) toneAcc->diffuse += d * luma(Lc);`
  （`Lc` は shadowFactor 込みの光色 `:112-113`、`d` は brilliance 適用後の N·L `:117-123`、
  `luma` は Rec.709。albedo `C`・`mat.diffuse`・`diffuseAo` は**含めない** — AO はレシピ側で
  contact/shape として独立に効かせ、素材色はトーンに漏らさない）。
- highlight ブロックの加算（`shading.hpp:173-178`）で
  `if (toneAcc) toneAcc->specular += luma(spec加算分);`（`specularCut` 用）。
- `shadePrincipled`: hard-shadow 枝は `ndl * shadowFactor * luma(l.color)`（`principled.hpp:184-222`）、
  area-light 枝は可視性がサンプルループ内のため `inv * accD * luma(l.color)`（`:223-269`）を積算。

### 5.2 hit_shader でのトーン合成

mesh 枝（`hit_shader.hpp:198-203` の AO 取得後）と primitive 枝（`:342-350`）の両方で:

```cpp
if (c.opt.hatch.enable) {
  const ToneRecipe& tr = c.opt.hatch.tone;
  const bool q = c.opt.aoEnhanced() || c.opt.aoWriteAov;   // quality AO gather ran
  const float cAo = q ? aoAov.contact : ao.openness;
  const float sAo = q ? aoAov.shape   : ao.openness;
  float t = tr.ambient + tr.diffuseWeight * toneAcc.diffuse;
  t *= std::pow(cAo, tr.contactAoPow) * std::pow(sAo, tr.shapeAoPow);
  if (tr.specularCut > 0.0f && toneAcc.specular > tr.specularCut) t = 1.0f;
  hs.hatchTone = t;   // clamp/remap は消費側 (6.5)
}
```

- `AoShade`（`ao_shade.hpp:30-35`）に `float openness = 1.0f;` を追加し、
  `aoApplyFactors` / `computeAoShade` / coarse-AO ルックアップ（`hit_shader.hpp:134` の
  `openness` 出力）から設定する。これで legacy AO パス（quality でない `computeAO`）でも
  トーンに AO が乗る。AO オフ（`aoSamples==0`）なら中立値 1 のまま＝拡散のみのトーン。
- `HitShade` に `float hatchTone = 1.0f;` を追加（`hit_shader.hpp:33-81`。
  **注意**: `PixelResult` の集約初期化は位置依存（`transparency.hpp:493-521`）なので、
  フィールド追加時は初期化リストを同時に更新する）。
- `PixelResult` に `float firstHatchTone = 1.0f; uint8_t firstHatchHit = 0;` を追加し、
  第一ヒット捕獲ブロック（`transparency.hpp:394-417`）で `hs.hatchTone` / 1 を取り込む。
- `storeShadingChannels()`（`embree_renderer.cpp:166-238`）の AO AOV 書き込み（`:226-233`）の
  後に `if (opt.hatch.enable) { res.hatchTone[pix] = pr.firstHatchTone;
  res.hatchMask[pix] = pr.firstHatchHit; }` を追加。adaptive AA の replicated 経路も
  この関数を通るため追加対応は不要（§2）。
- albedo 確保ゲート（`embree_renderer.cpp:504-507`）を
  `|| (opt.hatch.enable && (opt.hatch.base == HatchBase::Albedo || opt.hatch.ink == HatchInk::FromAlbedo))`
  で拡張し、書き込みゲート（`hit_shader.hpp:207-220` と primitive 対応箇所）にも同条件を足す。
- `fromEdge`（baked NPR 装飾、`hit_shader.hpp:335-341`）のヒットは AO・影なしで
  トーン＝ほぼ紙白になるが、`--keep-baked-edges` 併用時のみの周辺事象であり許容（注記のみ）。

### 5.3 二値化ノイズ対策（AO 側、指示書 §2.10）

優先順に:
1. CLI ハーネスで `--hatch` 時の既定を `aoResDiv = -1`（out。`ao_coarse.hpp` の
   セル毎ギャザー+バイラテラルはトーン専用デノイザとして機能する）、
   `aoLowDiscrepancy = on` にする。明示指定があれば尊重（`strokeThicknessSet` と同じ
   *Set フラグパターン、`cli.hpp:276-278` 参照）。※ `aoLowDiscrepancy` は
   `aoEnhanced()` トリガ（`render_options.hpp:79-82`）なので quality AO 経路にもなる。
2. **`aoResFallbackSppMul`（既定 4、AO 側オプション）** — フォールバック画素の追加サンプリング。
   フック点は棄却サイト唯一の `hit_shader.hpp:137-141`。`computeAoShade` は既に `sampleMul`
   引数を持つ（`ao_shade.hpp:132` → `:47` で乗算）ため、フォールバック呼び出しの
   `c.aoSampleMul` に乗じるだけ。シードは座標純関数（`ambient_occlusion.hpp:43,183`）なので
   決定性は保たれる。adaptive AA の `scBoost.aoSampleMul = coarseAoOn ? 1 : ss*ss`
   （`embree_renderer.cpp:1633-1639`）とは独立に合成される点をコメントで明記。→ Phase 3。
3. `toneSmooth` は前提（position ガイド）が崩れているため実装しない（§2, §10）。

まず 1 + ss=3〜4 で胡麻塩が実際に見えるかを確認してから 2 に進む（指示書 §2.10 の手順）。

---

## 6. `applyHatch` の実装

### 6.1 指示書擬似コードの欠陥と修正（数値検証済み）

- **F1 — フェード導入時の被覆ポップ**: `1 - smoothstep(w-0.5, w+0.5, d)` は `w→0, d→0` で
  **0.5** に飛ぶ（幅 0 の線が中心画素で半被覆から生まれる）。修正: 線は厳密な 1D 箱フィルタ重なり
  `cov = clamp((min(d+h, w) - max(d-h, -w)) / (2h), 0, 1)`（`h = halfAA = max(0.5, edgeSoftness)`、
  コスト同等・全幅で厳密）。点/穴はエネルギークランプ `att = min(1, A_p·r²/(4h²))` を乗算。
- **F2 — 素朴な 50% 反転は不連続**: 穴を `r50` 半径から始めると点にも穴にも属さない領域が残り、
  平均被覆が切替点で +9%（円）/ +17%（正方）ジャンプする。修正: 穴は**被覆半径**
  `R0 = 2^(1/p)·(0.5 + jitter)`（双対格子の任意点までの Lp 距離上界）から開始 →
  起動時に穴が平面全体を覆い `invCov ≡ 0` で厳密に連続。
- **F4 — `T[lv]` は K=0 でゼロ除算**: `t = (K==0) ? toneHi : toneHi + (toneLo-toneHi)·lv/K`。
- **F5 — `firstLevel` の `-j` は `INT_MIN` で UB**: `unsigned u = (unsigned)j; if (j<0) u = 0u-u;`。
- F3（補足）: K>0 ではレベル充填中（tone > toneLo）の平均被覆は原理的に `1-tone` を下回る
  （粗レベルが不足分を担うと後で縮む＝単調性違反になるため）。線形性テストは
  `tone ≤ toneLo − 1/fadeInv` の帯域のみで検証する。
- F6: 探索窓は摂動を含めて広げる（6.3）。
- F7: jitter>0 の Dot レイヤ内は `max` でなく `1−∏(1−c)` 合成（重なりの過小評価回避。
  どちらも単調性を保つので忠実度の選択）。

### 6.2 格子 / マーク / 摂動の 3 層分離（`npr/hatch_ink.hpp`, internal）

**格子（配置とレベル）** — kind 非依存の共有ロジック:

```cpp
inline int firstLevel(int j, int K) {          // F5 fixed
  if (j == 0 || K <= 0) return 0;
  unsigned u = (unsigned)j; if (j < 0) u = 0u - u;
  int ntz = 0; while (!(u & 1u)) { u >>= 1u; ++ntz; }
  return ntz >= K ? 0 : K - ntz;
}
// Line: lv = firstLevel(j, K)         (1D)
// Dot : lv = max(firstLevel(i, K), firstLevel(j, K))   (2D quadtree refinement)
// threshold: t = (K==0) ? toneHi : toneHi + (toneLo - toneHi) * lv / K   (F4 fixed)
// fade    : clamp((t - tone) * fadeInv, 0, 1)     … tone >= t なら不在
```

「一度現れたマークは消えない・動かない」は、位置が tone 非依存、fade/半径が tone の
単調非増加関数、レイヤ内合成が max / `1−∏(1−c)` であることから構成的に成立する。

**マーク形状（被覆率）**:

- Line: `u = -sin(a)·x + cos(a)·y`, `v = cos(a)·x + sin(a)·y`（沿線座標）。
  `w = 0.5·widthPx·fade·(摂動)`、被覆は F1 修正の箱フィルタ式。
- Dot: 中心 `(i·step, j·step)`+jitter。`d = Lp(dx', dy')`（dotAngleDeg 回転、
  `·√aspect / ÷√aspect` スケール後）。半径:
  `r = step·ρ_p·√(clamp(1-tone,0,1))·fade`、`ρ_p = 1/√A_p`,
  `A_p = (2Γ(1+1/p))²/Γ(1+2/p)`（p=1: 2, p=2: π, p≥16: L∞=4。起動時テーブル、
  `std::tgamma`）。1/√A_p スケールにより同一 tone の平均被覆が形状不変（指示書 §2.4.1 の
  面積補正）。K=0 では古典 AM スクリーンに厳密に退化（被覆 ≈ 1−tone）。
- 50% 反転（invertAbove50、F2 修正版）: `t50 = min(0.5, toneLo − 1/fadeInv)`
  （K>0 で全レベル充填前に反転しないためのガード。`t50 < 0.05` なら反転無効+警告）。
  `tone < t50` で: 点半径を切替時値に凍結し、双対格子 `((i+.5)·step, (j+.5)·step)`
  （独立 hash ストリームで jitter）の白穴を
  `r_hole = step·[ρ_p·√tone + (R0 − ρ_p·√t50)·(tone/t50)]` で縮小。
  `final = max(dotCov, 1 − max(holeMark))` — 単調・連続で、tone→0 で被覆が **1.0 に厳密到達**。

**摂動** — `markSeed(seed, layer, i, j, stream)` と沿線座標のみの純関数。**tone は絶対に入れない**:

```cpp
u += wobbleAmpPx * valueNoise1(markSeed(..., H_WOBBLE), v / wobbleWavePx);
w *= 1 + widthJitter * valueNoise1(markSeed(..., H_WIDTH), v / max(strokeLenPx, wobbleWavePx));
// stroke duty: phase = (v + strokeLenPx*u01(markSeed(...,H_STROKE))) mod (len+gap) → 台形 (taper)
// tooth: cov *= 1 - toothAmp * (0.5 + 0.5*valueNoise2(seed', x/toothScalePx, y/toothScalePx))
// jitter: off = (u01(markSeed(...,H_JITX)) - 0.5) * 2 * jitter * step   (X/Y 独立、穴は H_HOLEX/Y)
```

指示書の `noise(v/wavelength + hash(j))` から意図的に変更: `hash(j)` は座標に足さず
**シードに畳む**（大きな座標オフセットによる float 精度劣化の回避。意図は同一）。

### 6.3 正規化・探索窓・最小フィーチャガード

`normalizeHatchLayers()`（§4.1 から呼ぶ。`applyHatch` 単体呼び出しでも入口で適用）:

- 最小フィーチャ: `K_eff = min(K, floor(log2(spacingPx / 2)))` で実効 step ≥ 2px を保証
  （ss は最終解像度打ちなので救えない — 指示書 §3.7 の結論）。`spacingPx < 4` は 4 に
  クランプ+警告。検証可能: `spacing=10, K=5` と `K=2` の出力が byte-identical。
- 摂動クランプ: `jitter ≤ 0.5`, `widthJitter ≤ 1`, `dotAspect ∈ [0.5,2]`,
  `edgeSoftness ∈ [0,2]`, `wobbleAmpPx ≤ step`, `widthPx ≤ 2·step`。
- 探索窓（F6）: 線 `pad = wobbleAmpPx + 0.5·widthPx·(1+widthJitter) + halfAA` →
  `j0 = floor((u−pad)/step), j1 = floor((u+pad)/step)`。
  点 `pad = jitter·step + stretch·step·ρ_p·(反転時は √(1−t50)) + halfAA`
  （`stretch = 2^max(0,0.5−1/p) · max(√aspect, 1/√aspect)`）、穴は `R0` 基準。
  クランプ後は窓 ≤ 5 セル/軸が保証される。

### 6.4 hash / noise（`npr/hatch_ink.hpp` にローカル実装）

既存コードに再利用できる座標ハッシュ/ノイズは無い（§2）。Wellons lowbias32 +
hashCombine + ストリーム定数（H_JITX/H_JITY/H_WOBBLE/H_WIDTH/H_STROKE/H_TOOTH/H_HOLEX/H_HOLEY）、
1D/2D value noise（整数格子ハッシュ値の cubic 補間、[-1,1)）。スレッド状態・static・
累積順序を一切持たない座標純関数なので、TBB のタイル分割によらず bit-exact。

### 6.5 トーン処理・合成・コントラスト保証（`applyHatch` 本体）

画素ごと（TBB `parallel_for` 行分割。全計算が座標純関数なので決定性は自明）:

1. `m = mask[p]`; `m ≤ 0` なら**何もしない**（背景は fog/透過込みの既存値を保持。
   透過背景では premultiplied 値に触れない — 指示書 §2.13）。
2. トーン整形（全段が 1→1 を保存するので紙は厳密に無インク）:
   `t = clamp01((tone[p] − blackPoint) / (whitePoint − blackPoint))` →
   `t = pow(t, gamma)`（リニア域のアーティスティックカーブ）→ `t = srgbEncodeF(t)`
   （`image_ops.hpp:33`、リニアのまま閾値に掛けると中間調が潰れる）→
   `toneLevels > 1` なら `t = round(t·(N−1))/(N−1)`（表示域での量子化）。
3. 下地 `B` とインク `I`（表示空間）:
   - `mode == Over`: `B = 現 frame.color[p]`（base/ink の base 選択は無視）。
   - `mode == Ink`: `B = paperColor` または `srgbEncodeF(albedo[p])`
     （`albedoQuantize = N` なら `round(a·N)/N` — ベタらしさ）。
   - `I = inkColor` または albedo 由来（`FromAlbedo`: `srgbEncodeF(albedo)` を
     コントラスト保証で暗転させたもの）。
4. コントラスト保証（指示書 §2.7、実務上の要）:

```cpp
float lb = luma(B), li = luma(I);
if (lb - li < opt.inkMinContrast) {
  const float target = std::max(0.0f, lb - opt.inkMinContrast);
  if (lb >= opt.inkMinContrast) {
    const float s = (li > 1e-4f) ? target / li : 0.0f;
    I *= s;                                  // darken, hue-preserving
  } else {
    const float lt = std::min(1.0f, lb + opt.inkMinContrast);
    I = (li > 1e-4f) ? I * (lt / li) : Vec3{lt, lt, lt};   // 版画的: 明るいインクへ持ち上げ
  }
}
```

   インク側のみ動かす（下地を動かすと分子の色分けが崩れる）。暗い下地では持ち上げ決め打ち
   （オプションにしない — 指示書の指定）。
5. レイヤ合成（乗算 1 種類のみ。`I=0` では over と一致し、色インクでは重なりが `I²` に濃くなる）:

```cpp
float f[3] = {1, 1, 1};
for (layer L) {
  const float c = layerInk(L, x, y, t) * L.opacity;
  for (k) f[k] *= 1.0f - c * (1.0f - I[k]);
}
for (k) out[k] = B[k] * f[k];
```

6. マスクで境界 AA + 透過対応:
   `rgb' = rgb·(1−m) + out·m·(transparentBackground ? alpha : 1)`、alpha は変更しない。

---

## 6.5 マークスタイルのプリセット表（プロダクト仕様）

一次インタフェースは名前付きプリセット。個別パラメータは上書き。既定は **pen-cross**
（確率的パラメータゼロ＝seed 非依存で再現安定、3 層の段階的閾値で全階調をカバー、
反転機構が不要、4 通りの base/ink すべてで成立するため）。
想定出力 1000〜2000px。表記: 層 = {kind, 角度°, spacingPx, K, widthPx, toneHi, toneLo, fadeInv, opacity}。
未記載の MarkStyle は既定値。

| プリセット | レイヤ構成 | MarkStyle |
|---|---|---|
| **pen-cross**（既定） | L0 Line 45° 10 2 1.1 0.95 0.55 16 1.0 / L1 Line −45° 10 2 1.1 0.62 0.30 16 1.0 / L2 Line 0° 10 2 1.1 0.32 0.10 16 1.0 | edgeSoftness 0.5（硬い製図ペン、摂動なし） |
| pencil | L0 Line 55° 12 2 1.8 0.92 0.50 10 0.85 / L1 Line −35° 12 2 1.8 0.55 0.22 10 0.85 | edgeSoftness 1.2, toothAmp 0.25, toothScalePx 3, wobbleAmpPx 1.2, wobbleWavePx 36, widthJitter 0.35, strokeLenPx 26, strokeGapPx 6, strokeTaper 0.3 |
| engraving | L0 Line 0° 16 3 2.4 0.97 0.12 8 1.0 | edgeSoftness 0.5, widthJitter 0.5, wobbleAmpPx 0.5, wobbleWavePx 64 |
| stipple | L0 Dot — 10 2 — 0.96 0.35 12 1.0 | shapeExponent 2, jitter 0.4, invertAbove50 true, edgeSoftness 0.6 |
| screentone-60 | L0 Dot 45° 5 0 — 1.0 1.0 32 1.0 | shapeExponent 2, jitter 0, invertAbove50 true, edgeSoftness 0.5 |
| manga-square | L0 Dot 45° 6 0 — 1.0 1.0 32 1.0 | shapeExponent 16, jitter 0, invertAbove50 true, edgeSoftness 0.5 |

spacing の根拠: pen-cross 10px（図全体で 100〜200 本、最細 step 2.5px でガードを回避）。
pencil 12px（1.8px 幅のストロークに空気を残す）。engraving 16px・K3（最細 step ちょうど 2px、
単方向で全階調を担う）。stipple 10px・K2（中間調が滑らか、モアレ回避）。
screentone-60 5px（300dpi・5 インチ幅の図で 60 lpi 相当の古典 AM スクリーン）。
manga-square 6px（100% 表示で正方形要素が読める粗さ）。

pen-cross の最暗部被覆は約 0.8 で頭打ち（ペン画として正しい挙動）。被覆 1.0 到達の
回帰テストは invertAbove50 付き Dot プリセットを対象にする。

---

## 7. CLI（`--edges` パターン踏襲）

Phase 1:

```
--hatch <on|off>                tone hatching NPR shading [off]
--hatch-mode <ink|over>         ink: discard shaded color / over: hatch over it [ink]
--hatch-preset <name>           pen-cross|pencil|engraving|stipple|screentone-60|manga-square
                                [pen-cross]  (Phase 1 implements pen-cross)
--hatch-ink-color <#RRGGBB>     ink color, DISPLAY-encoded [#000000]
--hatch-paper-color <#RRGGBB>   paper color, DISPLAY-encoded [#FFFFFF]
--hatch-base <paper|albedo>     Ink-mode base [paper]
```

Phase 2: `--hatch-layer <idx:key=val,...>`（個別上書き）。
Phase 3: `--hatch-style <ID=spec>`（per-section。先頭 `=` 分割・`_show` 除去・
`groupNames` 解決・`--list-groups` 発見 UX を `--edge` `cli.cpp:790-810` /
`scene_setup.cpp:358-375` から流用）、`--hatch-ink <fixed|albedo>`、
`--hatch-tone-*`（ToneRecipe）、`--ao-res-fallback-mul <N>`。

- `Options`（`cli.hpp`）に `hatch*` フィールド群を追加し、`applyShadingOptions`
  （`scene_setup.cpp:436-645`）で `ropt.hatch` へ写す。
- **NPR AO 既定**（§5.3）: `--hatch on` かつ AO 有効（`aoSamples > 0`）のとき、明示指定が
  なければ `aoResDiv = -1`・`aoLowDiscrepancy = true`（`aoResSet`/`aoLdSet` フラグで判定）。
- `--quality` には NPR 段を**追加しない**（§2 の Discrepancy 参照。gi=true を焼き込む
  現行 3 段と衝突する。マークプリセット `--hatch-preset` は品質でなくスタイル軸 —
  `docs/quality_presets.md` §4 の分類と整合）。
- usage 文字列（`cli.cpp:1130-1331`）の edges ブロックの後に hatch ブロックを追加。

---

## 8. テスト（`tests/test_hatch.cpp`、`tests/CMakeLists.txt` に 3 行パターンで登録）

回帰として押さえるもの（指示書 §4 の必須項目 + 欠陥修正の検証）:

1. **既定 OFF の byte-identical**: `hatch.enable=false`（他の hatch フィールドを非既定値に
   しても）のレンダが素の既定レンダと `framesEqual` で一致。加えて `enable=true` では
   異なることの非自明性チェック（`test_object_space_edges.cpp:695-705` のパターン）。
2. **入れ子性（単調性）**: `applyHatch` を直接叩き、合成トーン（水平グラデーション＋一様値
   スイープ）で「tone を下げると任意ピクセルのインクが単調非減少」。`firstLevel` の
   単体チェック（j 奇数 → K、j = 2^K·m → 0）も併置。
3. **ss 不変の線幅**: ss=1 と ss=4 のレンダで線幅（暗画素比率）が一致（±1px 許容）。
4. **4 通りの base/ink でインク可視**（コントラスト保証の検証。Phase 1 は Paper/Albedo ×
   Fixed の 2 通り、Phase 3 で 4 通りに拡張）: 淡黄インク×紙白、濃紺下地×黒インクでも
   ハッチ領域の分散が非ゼロ。
5. **Lp 面積補正**: `shapeExponent ∈ {1,2,4,16}` で同一 tone の平均被覆が一定（±数%。
   `tone ≤ toneLo − 1/fadeInv` の帯域で測定 — F3）。→ Phase 2
6. **摂動下の単調性**: jitter / wobble を入れても 2 のテストが通る（摂動が tone 非依存で
   あることの検証）。→ Phase 2
7. **反転で被覆 1.0 到達**: invertAbove50 の Dot レイヤで tone→0 のとき被覆が 1.0 に到達し、
   切替点で平均被覆が連続（F2 修正の検証）。→ Phase 2
8. **決定性**: `tbb::global_control` の scoped block（`test_ao_coarse.cpp:302-319` パターン）で
   1 スレッドと N スレッドが bit-exact（hatch + AO 有効レンダ）。
9. **K_eff ガード**: `spacing=10, K=5` と `K=2` の `applyHatch` 出力が byte-identical。

レンダ系テストは `tests/render_test_util.hpp` の合成シーン（makeQuad / makeMaterialSphereScene）
を使う（LFS 回避）。`data/1ab0_scene4_toon1.pov` は目視確認専用（`.inc` が LFS のため
テストに使うなら `needs_lfs` ラベルが必要 — 使わない）。

---

## 9. 段階分け（各フェーズで「動く絵」+ `task test` 緑）

- **Phase 1 — 最小の動く絵**（本ブランチで実装）:
  `hatch_types.hpp`（全型を確定、公開 3 点同期）/ FrameResult AOV / ToneAccum タップ
  （shadeLocal + shadePrincipled）/ hit_shader・transparency・store 結線 / パイプライン
  正規化・downsample・applyHatch 呼び出し / applyHatch（**Line のみ・摂動なし**・
  pen-cross 既定・Ink|Over・Paper|Albedo×Fixed・乗算合成・コントラスト保証・K_eff ガード・
  F1 修正箱フィルタ・TBB）/ CLI（§7 Phase 1 分）/ テスト 1,2,3,8,9 + Paper/Fixed 可視。
- **Phase 2 — マークスタイル**: Dot（Lp・面積補正・F2 修正反転・jitter）/ 線摂動
  （wobble・widthJitter・stroke 途切れ・taper・tooth）/ hash+noise / プリセット 6 種 /
  `--hatch-layer` / テスト 5,6,7。
- **Phase 3 — セクション別と仕上げ**: FromAlbedo インク + albedoQuantize /
  `Scene::groupHatchStyle` + `hatchGroup` AOV（hi-res 保持、applyHatch がセル代表サンプルで
  参照）+ `--hatch-style ID=spec` / ToneRecipe CLI / `aoResFallbackSppMul` /
  `docs/api/libumbreon.md`・`docs/quality_presets.md` 追記 / テスト 4 の 4 通り化。

---

## 10. 非目標

- 物体空間 UV パラメータ化（Praun の lapped texture）。`hatchUv` は「null ならスクリーン座標」
  という関数分岐の拡張点のみ用意（§3.1）。UV の画面微分で step をスケールすれば遠景の
  線詰まりも同じ関数内で処理できる — 設計メモとして記録。
- TAM テクスチャ生成、アニメーションの時間的コヒーレンス、GPU 実装、学習ベース手法。
- 重み付き Voronoi 等の最適化ベースのスティップル配置（§6.2 の jitter で代替 —
  Secord/Deussen 系が何を近似しているかの参照先として文献のみ挙げる）。
- `toneSmooth`（SVGF ラッパ）— ガイド AOV（`frame.position`）が GI 限定で前提が崩れるため
  将来課題に格下げ（§2 Discrepancy、§5.3）。
- fog をトーンに畳む表現（silhouette 線側の変更を伴う）。

---

## 参考文献

- Praun, Hoppe, Webb & Finkelstein, *Real-Time Hatching*, SIGGRAPH 2001 — TAM の入れ子性
- Webb, Praun, Finkelstein & Hoppe, *Fine Tone Control in Hardware Hatching*, NPAR 2002 — fade の元ネタ
- Freudenberg, Masuch & Strothotte, *Real-Time Halftoning*, EGRW 2002 — 閾値方式
- Winkenbach & Salesin, *Computer-Generated Pen-and-Ink Illustration*, SIGGRAPH 1994
- Tarini, Cignoni & Montani, TVCG 2006 — 分子可視化で AO が形状知覚に効く根拠（ToneRecipe が AO 中心である理由）
- Ostromoukhov & Hersch, *Artistic Screening*, SIGGRAPH 1995 — マーク形状差し替えの先行例
- Secord, *Weighted Voronoi Stippling*, NPAR 2002 / Deussen et al., *Floating Points*, NPAR 2000

### Critical Files for Implementation

- `src/umbreon/render/hatch_types.hpp`（新規・公開）
- `src/umbreon/npr/hatch_shade.hpp`（新規・公開）/ `hatch_shade.cpp` / `hatch_ink.hpp`（新規・internal）
- `src/umbreon/render/render_options.hpp` / `frame_result.hpp` / `render_types.hpp`
- `src/umbreon/render/pipeline.cpp` / `embree_renderer.cpp` / `progress_cost_model.hpp`
- `src/umbreon/shading/shading.hpp` / `principled.hpp` / `hit_shader.hpp` / `transparency.hpp`
- `src/umbreon/ao/ao_shade.hpp`
- `src/bench/cli.hpp` / `cli.cpp` / `scene_setup.cpp`
- `CMakeLists.txt` / `tests/CMakeLists.txt` / `tests/test_hatch.cpp`（新規）
- `docs/api/libumbreon.md` / `docs/plans/README.md`
