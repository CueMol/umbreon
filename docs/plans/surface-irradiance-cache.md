# umbreon Adaptive Surface Irradiance Cache + contact/shape AO + OIDN 実装プラン

## Context

umbreon に **one-bounce diffuse GI** を入れる。全ピクセルで GI ray を飛ばすのではなく、表面上の少数代表点(cache record)でのみ irradiance を計算して cache し、shading 時に近傍 record を補間再利用する **irradiance caching**(Ward–Heckbert 1988/1992, Křivánek 2006)。低周波の indirect は cache、細部の締めは **ray-traced contact/shape AO**(別プラン参照)、残留ノイズは **denoiser**(自前 edge-aware à-trous を既定、OIDN を optional backend)で処理する。

**前提(土台)**: [ao-quality.md](ao-quality.md) の AO infra をそのまま使う。具体的には:
- `computeAOQuality` → `AOResult{ contact, shape, openness, bent, avgHitDist }`(contact/shape 分離、bent normal、距離減衰)。
- first-hit / G-buffer AOV(`position`(本プランで追加)/`normal`/`albedo`/`depth`/`objectId`)を主経路で出力するゲート(ao-quality.md フェーズ5)。
- 2色 sky/ground グラデーション directional ambient(`bent` 由来) = 本プランの **environment fill**。
→ ao-quality.md フェーズ1–2 + フェーズ5 を本プランの**フェーズ0(前提)**とする。AO 単体プランの上に cache + denoiser を積む構成。

**実体環境**(grounding 確定):
- Intel Embree 4.4.1 + oneTBB 直叩きの自前 C++17 レンダラ。依存は Embree + TBB のみ(`CMakeLists.txt:53-103`)。
- **1シーン1フレーム**・temporal/animation 状態なし(`render()` は毎回 BVH を再構築、`embree_renderer.cpp:204-205`、`embree_renderer.hpp:17-25`)。→ 時間的再利用は不要、cache は単フレーム使い切り。
- **決定性契約**: サンプリングは hi-res `(px,py,sample)` 由来の `tea2` のみ。TBB スレッド数非依存・run-to-run bit 一致を**テストで固定**(`test_render.cpp:858-876`、raw `!=` 比較)。
- レンダ後も `device_`/`scene_` を生かす設計(post-pass の BVH query 用、`embree_renderer.cpp:362-366`)→ cache fill / interpolation のレイキャストはこの live BVH を使える。
- HDR/EXR writer は無い(PNG sRGB + PPM のみ、`image_io.cpp:148-151`)。color buffer は linear HDR、`writeImage` が sRGB encode。

**Scope**: cache + GI は **mesh hit のみ**(AO と同じゲート)。outline/VdW primitive(sphere/cylinder)は flat silhouette で GI ライティング対象外(`hit_shader.hpp:144-146` の `aoFactor=1.0f` リテラルゲートを踏襲)。

**目標**: 物理的に厳密な path tracing ではなく、「radiosity 近似の低周波 indirect + AO の形状ディテール + denoise の仕上げ」。POV-faithful な非物理ヘリテージに合わせ、正規化定数はユーザ gain で吸収する。

実装状況: **提案・未着手**。

---

## 全体アーキテクチャ(5パス + denoise)

```
[A] Primary G-buffer pass     first hit → position/normal/albedo/depth/objectId(=group)/materialId
                              + L_direct(既存の direct diffuse/spec + shadows)
        │
[B] Record placement pass     決定的に cache record を表面へ配置(seed + neighbor clamp)
        │  (single-thread, canonical order)
[C] Record fill pass          各 record で one-bounce irradiance を hemisphere gather
        │  (TBB 並列, per-record 固定seed)  → E_i, R_i, ∇E_i, bentNormal_i
        │
[D] Interpolation pass        各 shading point で近傍 record を gather・leak 除去・補間 → E_cached(x)
        │  (TBB 並列, read-only)
[E] Contact/shape AO + 合成    L = L_direct + (kd·E_cached + ambient·ambLightDir)·C·shapeAO ; L *= contactAO
        │
[F] Denoise (optional)        AtrousBilateral(既定/自前) | OIDN(build-option) | None
                              linear HDR を guide(albedo/normal/depth)付きで処理
        │
   box-downsample → tone/gamma → sRGB write
```

**[A][E] は既存パイプライン内**(`integratePixel`/`shadeHit`/`shadeLocal`)。**[B][C][D]** は render() 後・downsample 前に走る新パス(live BVH 利用、`umbreon.cpp` の fog/edge と同じ post-pass 位置)。**[F]** は downsample 後・gamma 前。

---

## 決定性戦略(本プラン最重要)

研究が標準とする「マルチスレッド shading 中の lazy on-demand record 挿入」は**順序依存**(どのスレッドが未被覆点に先に到達するかで record 配置が変わる)で、本レンダラの bit-exact 契約を破る。**採用しない**。代わりに3分割:

1. **[B] Placement = 決定的・順序非依存**。record 集合を shading 順序と無関係に確定する。
   - **既定: visible-surface voxel seeding**。hi-res primary G-buffer の各 hit を `position` から世界座標 voxel(target spacing = 局所 `avgHitDist` または scene diagonal/N で sizing)へ落とし、**未占有 voxel に1 record**。voxel 占有は順序非依存(占有/非占有の集合演算)→ 完全決定的・並列化可。
   - **adaptive 細分(任意)**: fill 後に `R_i` が小さい(高ディテール)領域の voxel を1段細分し2巡目を追加。各巡は順序非依存。
   - **代替: per-vertex seeding**(welded/indexed mesh の頂点、`scene.hpp:144-150` `posClass`)= view 非依存 cache。voxel seeding と排他で選択可。
2. **[C] Fill = 並列・record 独立**。`E_i` 等を TBB 並列計算。**各 record は自分のインデックス/座標 hash から固定 RNG seed**(共有 atomic カウンタ不可)。fill 中に record 間参照なし → データ競合なし・bit 一致。neighbor clamping は `min` 演算で順序非依存(commutative)なので fill 後にまとめて適用可。
3. **[D] Interpolation = read-only**。cache は不変。各 shading point は独立に近傍 gather・補間。完全並列・決定的。

placement は単スレッド canonical order(voxel seeding なら順序すら不問)だが O(record数) で安価。重い hemisphere gather は [C] で並列化する。→ **「安価な決定的配置」と「重い並列 fill」を分離**するのが要。

Embree レイは決定的(順序非依存最近接)。incoherent な GI/AO ray は `rtcIntersect1`/`rtcOccluded1` 単発 + `RTC_RAY_QUERY_FLAG_INCOHERENT` が最速(Embree 公式)。record lookup は **Embree scene ではなく自前 hash-grid**(record は geometry でないため安価)。

---

## データ構造

```cpp
// secondary_rays.hpp / 新規 irradiance_cache.hpp(detail, Embree込み・非公開)
struct IrradianceRecord {
  Vec3  position;      // 表面上の record 位置(cache 空間キー)
  Vec3  normal;        // record 法線
  Vec3  irradiance;    // RGB one-bounce indirect irradiance E_i
  Vec3  bentNormal;    // 平均非遮蔽方向(env fill / lookup 方向)
  Vec3  gradT[3];      // 並進勾配 ∂E/∂{x,y,z}(任意、フェーズ4で配線; 既定ゼロ)
  Vec3  gradR[3];      // 回転勾配(任意)
  float radius;        // 有効半径 R_i(harmonic mean 距離 → clamp)
  uint32_t geomID;     // 漏れ除去用
  uint16_t componentID;// = groupForTri()(CueMol section)。別 component またぎ補間を拒否
};

// 自前空間構造(決定的・並列 fill 可)。voxel = 最小 a·R_i 相当。
struct IrradianceCache {
  std::vector<IrradianceRecord> records;
  // hash-grid: cell → record index 群。record は半径 a·R_i が跨る全 cell に登録。
};

// denoiser backend(F)
enum class DenoiserBackend { None, AtrousBilateral, OIDN };
```

**RenderOptions 追加**(全て既定=現状再現、master gate で byte-identical):
```cpp
// --- diffuse GI (surface irradiance cache) --- 既定 OFF = 現状の local shading
bool  gi              = false;   // MASTER gate。false => 全パス無効・byte-identical
int   giSamples       = 64;      // record あたり hemisphere gather 本数
float giMaxDistance   = 0.0f;    // 0 => scene 依存自動(diagonal*係数)。indirect ray の tfar
float giIntensity     = 1.0f;    // indirect の gain(POV 非物理ヘリテージの正規化吸収)
float giAccuracy      = 0.15f;   // 補間精度 a。w_i>1/a で採用、max 有効半径=a·R_i
float giRecordSpacing = 0.0f;    // 0 => 自動。voxel seeding の target 世界間隔
bool  giGradients     = false;   // 並進/回転勾配で線形補間(滑らか・record減)
float giNormalReject  = 0.85f;   // dot(n_x,n_rec) 下限(下回ると棄却)
bool  giComponentReject = true;  // componentID 不一致 record を棄却(leak 防止)
bool  giSeedPerVertex = false;   // true => mesh 頂点 seed(view非依存)。既定は voxel seed
// --- denoise ---
int   denoiser        = 0;       // 0=None,1=AtrousBilateral,2=OIDN(DenoiserBackend)
int   denoiseIters    = 5;       // à-trous 反復(SVGF 既定)
float denoiseSigmaZ   = 1.0f;    // 深度 edge-stop σ_z
float denoiseSigmaN   = 128.0f;  // 法線 edge-stop σ_n
float denoiseSigmaL   = 4.0f;    // 輝度 edge-stop σ_l
bool  denoiseDemodulateAlbedo = true; // irradiance=color/albedo を処理し再乗算
bool  oidnCleanAux    = true;    // OIDN: primary-hit aux はほぼ無ノイズ → clean 既定
```

**FrameResult 追加**(ao-quality.md の AOV に追加):
```cpp
std::vector<float> position;   // W*H*3 world-space first-hit 位置(cache/denoise guide)
std::vector<float> indirect;   // W*H*3 補間 indirect(debug/denoise demodulation 用)
// albedo/normal/depth は ao-quality.md フェーズ5 で主経路化済みを流用
```

---

## パス別詳細

### [A] Primary G-buffer pass(既存の一般化)

ao-quality.md フェーズ5 で主経路化する first-hit 出力に **`position`(world)** を追加(`integratePixel` の first-hit ブロック `transparency.hpp:118-126` で `org + rd*nearDepth` を保存)。`L_direct` は既存 `shadeLocal` の direct diffuse/spec + shadows をそのまま使う(GI on でも direct は不変)。

### [B] Record placement(決定的)

```
spacing = giRecordSpacing>0 ? giRecordSpacing : sceneDiagonal * k0   // 例 k0≈0.01
for each hi-res primary mesh hit (raster order だが voxel 占有は順序不問):
    cell = floor(position / spacing)
    if cell 未占有 in seedGrid:
        seedGrid.mark(cell)
        records.push({ position, normal, geomID, componentID=groupForTri })
```
- `giSeedPerVertex` 時は mesh 頂点(`positions`/`normals`、`posClass` で溶接済み)を seed。
- placement では `R_i` 未確定。fill 後に neighbor clamping(下記)で確定するため、ここでは座標と法線のみ確定。

### [C] Record fill(並列・決定的)

各 record で one-bounce diffuse irradiance を hemisphere gather(`computeAO` と同じ origin offset / self-intersection eps / `cosineSampleHemisphere`、ただし `rtcOccluded1`→`rtcIntersect1` で最近接 hit を取得):
```
seed = hash(recordIndex)              // per-record 固定。共有カウンタ不可
f = frameFromNormal(record.normal)
E = 0 ; bentAccum = 0 ; invDistSum = 0 ; nHit = 0
for s in 0..giSamples-1:
    (u1,u2) = LD or tea2(seed, s)
    wi = cosineSampleHemisphere(u1,u2,f)         // cosine weight が 1/cos を相殺
    hit = intersectNearest(O, wi, eps, giMaxDistance)
    if hit:
        y = hit.position
        E += albedo(y) * directLightingAt(y)     // y で direct light 再評価(1 bounce)
        invDistSum += 1/hit.dist ; nHit++
        // bentNormal は「開いた方向」なので遮蔽 hit は寄与させない
    else:
        E += environmentRadiance(wi)             // = sky/ground グラデーション(ao-quality.md)
        bentAccum += wi
record.irradiance = E / giSamples
record.bentNormal = safeNormalize(bentAccum, record.normal)
record.radius     = clamp(nHit>0 ? nHit/invDistSum : giMaxDistance, Rmin, Rmax)  // harmonic mean
// giGradients: gather 中に並進/回転勾配(Ward–Heckbert/Křivánek)を同時蓄積
```
- `directLightingAt(y)`: y の法線・material で既存 light loop を1回評価(shadow 込み可)。これが1バウンス分。
- **harmonic mean 半径** `R_i = N_hit / Σ(1/r_k)`(近接面が支配 → 凹部で record 密)。`Rmin/Rmax` で clamp(過小爆発・過大外挿を防止)。

fill 後、**neighbor clamping**(Křivánek 2006、leak 防止の要)を順序非依存に適用:
```
for each record i, for each neighbor j in hash-grid:
    R_i = min(R_i, R_j + ||x_i - x_j||)         // min は可換 → 並列/順序非依存
```
分子表面(薄い突起・近接面・微小クラック)で **別面またぎの light leak が最大リスク**。harmonic mean + neighbor clamping + 下記 [D] の component/normal 棄却で多重防御。

### [D] Interpolation(read-only・並列)

各 shading point `x`(primary mesh hit)で:
```
S = {} ; wSum = 0 ; Esum = 0
for each record i in hash-grid near x:
    if giComponentReject && componentID_i != componentID_x: continue   // 別 section 棄却
    if dot(n_x, n_i) < giNormalReject: continue                        // 法線乖離棄却
    w_i = 1 / ( ||x - x_i|| / R_i + sqrt(max(0,1 - dot(n_x,n_i))) )
    if w_i <= 1/giAccuracy: continue                                   // 影響圏外
    E_i' = giGradients ? (irr_i + (n_i×n_x)·gradR_i + (x-x_i)·gradT_i) : irr_i
    Esum += w_i * E_i' ; wSum += w_i
E_cached = wSum>0 ? Esum/wSum : fallback(directional ambient only)
```
- 信頼 record が無い点(`wSum==0`)は directional ambient のみへフォールバック(穴を作らない)。決定的(placement で被覆を担保するが保険)。
- 任意で短い visibility check(x↔record の `rtcOccluded1`)を leak 棄却に追加可(コスト増、既定 OFF)。

### [E] Contact/shape AO + 最終合成

ao-quality.md の `computeAOQuality` で `contact`/`shape`/`bent` を取得。`shadeLocal` の ambient 項を indirect 項へ拡張(GI on のときのみ。off は既存式で byte-identical):
```
ambLightDir = sky/ground gradient(bent 由来)              // env fill(ao-quality.md フェーズ2)
indirect = (giIntensity * kd * E_cached + mat.ambient * ambLightDir) * C
L = L_direct + indirect * shapeAO                          // shape は indirect/ambient に
L *= contactAO                                             // contact は最終に弱く
```
- 研究の `L = L_direct + kd/π·E·shapeVis + bentEnv ; L *= contactAO` に対応。`kd=mat.diffuse`、1/π は `giIntensity` に吸収(POV 非1/π 慣習)。
- **AO は indirect を黒く潰す道具でなく形状ディテールの締め**。`contactAO` は小半径・弱 intensity、direct は強く潰さない(研究の指針)。`AO = 0.7·contact + 0.3·shape` 的初期値はヘッダ定数で。
- `indirect` を FrameResult.indirect に保存(denoise demodulation / debug)。

### [F] Denoise(optional backend)

`DenoiserBackend` を1インターフェース化、入力は **linear HDR color + guide(albedo[0,1] / normal[-1,1] / depth)**。downsample 後・gamma 前の最終解像度で実行(supersample box-average が一次デノイズ、本パスは仕上げ)。

**AtrousBilateral(既定・自前・依存ゼロ)** — Dammertz 2010 à-trous + SVGF edge-stop:
```
irr = denoiseDemodulateAlbedo ? color / max(albedo, eps) : color   // albedo 復調
for i in 0..denoiseIters-1:                                        // 既定5
  spacing = 1<<i                                                   // 2^i dilation(穴あき)
  kernel  = B3 spline {1/16,1/4,3/8,1/4,1/16} の 5x5 outer product
  for each pixel p:
    w(p,q) = w_z * w_n * w_l
      w_z = exp(-|z_p - z_q| / (σ_z*|∇z_p·(p-q)| + ε))
      w_n = max(0, dot(n_p,n_q))^σ_n
      w_l = exp(-|l_p - l_q| / (σ_l*sqrt(g3x3(Var(l_p))) + ε))     // 単フレーム→空間分散推定
    irr'(p) = Σ kernel(q)·w(p,q)·irr(q) / Σ kernel(q)·w(p,q)
out = denoiseDemodulateAlbedo ? irr * albedo : irr
```
既定 σ_z=1 / σ_n=128 / σ_l=4(SVGF 全シーン推奨)、ε≈1e-3。単フレームなので luminance variance は temporal でなく 7x7 空間 bilateral 推定。

**OIDN(optional・build-option)** — CPU device, "RT" filter, HDR:
```
oidnNewDevice(OIDN_DEVICE_TYPE_CPU); oidnCommitDevice
// aux が無ノイズなら prefilter 省略可。clean を明示する場合のみ:
beauty = oidnNewFilter("RT")
oidnSetSharedFilterImage(beauty,"color",  color,  FLOAT3,W,H,0,0,0)
oidnSetSharedFilterImage(beauty,"albedo", albedo, FLOAT3,W,H,0,0,0)   // [0,1]
oidnSetSharedFilterImage(beauty,"normal", normal, FLOAT3,W,H,0,0,0)   // [-1,1]
oidnSetSharedFilterImage(beauty,"output", out,    FLOAT3,W,H,0,0,0)
oidnSetFilterBool(beauty,"hdr", true)
oidnSetFilterBool(beauty,"cleanAux", oidnCleanAux)   // primary-hit aux はほぼ無ノイズ
oidnCommitFilter(beauty); oidnExecuteFilter(beauty)
oidnGetDeviceError(...) で検査; release
```
- primary-hit の albedo/normal は本質的に無ノイズなので `cleanAux=true` を prefilter なしで使える(ノイズ aux だと残留ノイズが乗るため、無ノイズ保証時のみ)。
- **denoise は tone-map 前の linear HDR に対して**(OIDN HDR 設計 / SVGF 順序)。
- OIDN は色だけ差し替え、guide buffer は AtrousBilateral と共有 → backend は drop-in 互換。

---

## 変更ファイル(中核)

- **新規** `src/umbreon/render/irradiance_cache.hpp`(detail, 非公開) — `IrradianceRecord`/`IrradianceCache`/hash-grid、`buildIrradianceCache(...)`([B]+[C]+neighbor clamp)、`interpolateIrradiance(x,n,comp,geom)`([D])。
- **新規** `src/umbreon/render/denoise.hpp` / `denoise_atrous.cpp` — `DenoiserBackend`、`denoiseAtrous(...)`(自前)、`denoiseOidn(...)`(`#ifdef UMBREON_HAVE_OIDN`)。bench でなく `umbreon` lib 側(`bench_core` は依存なし契約)。
- `src/umbreon/render/secondary_rays.hpp` — fill 用 `gatherIrradiance(...)`(`computeAO` 隣、`rtcIntersect1` 版)。`computeAOQuality` は ao-quality.md。
- `src/umbreon/render/shading.hpp`(行47-55) — ambient 項を indirect 項へ拡張(GI on 経路)。off は既存式維持。
- `src/umbreon/render/hit_shader.hpp`(行85-92) — `position` first-hit 出力、GI on 時に `interpolateIrradiance` を呼ぶ配線。
- `src/umbreon/render/transparency.hpp`(行118-126) — first-hit `position` 保存。
- `src/umbreon/umbreon.cpp`(行96-172) — post-pass 順序に [B][C][D] cache build → [F] denoise を挿入(fog/edge と downsample の間)。`EmbreeRenderer` の live BVH を渡す。
- `src/umbreon/render/render_types.hpp` — `RenderOptions`/`FrameResult` 上記追加。
- `CMakeLists.txt`(Embree/TBB block 行50-103 隣) — `option(UMBREON_WITH_OIDN OFF)` + `find_package(OpenImageDenoise CONFIG)` + `UMBREON_HAVE_OIDN` guard + `umbreon` への optional link(行131)。`bench_core` には足さない。
- `src/bench/cli.{hpp,cpp}` / `src/bench/main.cpp` — CLI フラグ(`parseBool`/`parseHexColor` 再利用)、`--dump-aov` に position/indirect 追加。
- `tests/test_render.cpp`(AO 回帰 行754-876 流儀) — 新テスト群。

---

## bit-exact / 決定性 戦略

1. **master gate `gi==false`** で全パス無効 → color byte-identical(既存全テスト無改変通過)。`denoiser==None` 既定。
2. **placement 順序非依存**(voxel 占有集合)+ **fill per-record 固定 seed** + **neighbor clamp は min**(可換) + **interpolation read-only** → TBB スレッド数非依存・run-to-run bit 一致。AO の決定性テスト(`test_render.cpp:858-876`)と同形の cache 決定性テストで固定。
3. GI on でも **direct 項は不変**(indirect は加算項、contact/shape は乗算項)。off↔on の差分は新項のみ。
4. **outline ゲート維持**: GI/AO は mesh 経路限定。outline 分岐は `aoFactor=1` リテラル・indirect 非加算で従来一致。
5. denoise は color のみ後段で書き換え、`denoiser==None` で no-op → 既定 byte-identical。OIDN 不在ビルドでも `denoiser==OIDN` 指定時は warning + AtrousBilateral フォールバック(ビルド差で出力が壊れない)。

---

## テスト計画(tests/test_render.cpp、5x5 ortho 流儀)

- **既定 bit-exact**: `gi=false`/`denoiser=None` で全既存ロック値一致(`approx 1e-6`)。
- **GI gate 決定性**: `gi=true` 同一シーン2回 → color raw `!=` で bit 一致(AO 流儀)。
- **indirect の方向性**: 白床+赤壁の近接で、床の壁側 channel が反対側より明るい(color bleeding が出る)を不等式 assert。
- **leak 棄却**: 薄い仕切りを挟む2 component で、`giComponentReject=on` のとき片側の明色が反対側へ漏れない(off 比で漏れ減)を不等式 assert。
- **半径 clamp/neighbor clamp**: 単一被覆 record の有効範囲が `a·R_i` を超えないこと、隣接 clamp 後に過大 `R_i` が縮むこと。
- **contact/shape 分離 + 合成**: indirect に shape、最終に contact が掛かる経路を、shape=0 で indirect 消失・contact=0 で全体暗化の不等式で確認。
- **denoise no-op**: `denoiser=None` で color 不変。`AtrousBilateral` でノイズ分散が低下しエッジ(深度/法線跨ぎ)が保たれる(平坦領域分散↓、エッジ近傍の片側平均が混ざらない)を assert。
- **OIDN(ビルド時のみ)**: `UMBREON_HAVE_OIDN` ガード下で実行、未定義時はテスト skip + フォールバック warning を確認。

---

## CLI フラグ追加

`--gi on|off`, `--gi-samples <n>`, `--gi-max-dist <world>`, `--gi-intensity <f>`, `--gi-accuracy <a>`, `--gi-spacing <world>`, `--gi-gradients on|off`, `--gi-normal-reject <cos>`, `--gi-component-reject on|off`, `--gi-seed-per-vertex on|off`, `--denoiser none|atrous|oidn`, `--denoise-iters <n>`, `--denoise-sigma-z|n|l <f>`, `--denoise-demodulate on|off`, `--oidn-clean-aux on|off`。`printUsage` と `main.cpp` の `ropt.* = opt.*` 転写に追記。

---

## ビルド(OIDN を optional 依存に)

- `CMakeLists.txt` に `option(UMBREON_WITH_OIDN "Enable Intel OIDN denoiser backend" OFF)`。ON 時のみ `find_package(OpenImageDenoise 2 CONFIG)`、見つかれば `target_compile_definitions(umbreon PRIVATE UMBREON_HAVE_OIDN)` + `target_link_libraries(umbreon PUBLIC OpenImageDenoise)`。
- OIDN 2.3.x / Apache-2.0 / CPU device 既定対応。依存は oneTBB(既存) + oneDNN。RT weights 同梱で lib は中量級 → **optional 必須**。`OIDN_FILTER_RT` のみ有効化で縮小可。
- 既定 OFF で従来通り Embree+TBB のみ。AtrousBilateral は依存ゼロで常時利用可 → OIDN 無しでも denoise 機能は成立。

---

## 実装順(各コミット独立 green、毎回 bit-exact 担保)

0. **前提**: ao-quality.md フェーズ1–2 + フェーズ5(AOresult contact/shape/bent、first-hit/AOV 主経路化)を先行実装。本プランの土台。
1. `position` first-hit AOV 追加 + FrameResult 拡張(未配線・byte 一致)。
2. `irradiance_cache.hpp`: 構造体 + hash-grid + 決定的 placement([B])。record 集合の決定性テスト(seed 再現)。
3. `gatherIrradiance` + parallel fill([C]) + harmonic-mean 半径 + neighbor clamp。fill 決定性テスト(per-record seed, bit 一致)。
4. interpolation([D]) + leak 棄却 + `shadeLocal` indirect 配線([E]) + `--gi` gate/CLU。color bleeding / leak / 決定性テスト。視覚確認。
5. contact/shape AO 合成統合([E] 完成) + env fill 配線。
6. 勾配補間([C]/[D] の `giGradients`) — 滑らかさ/record 削減の quality 検証。
7. denoise: AtrousBilateral([F] 自前) + CLI + no-op/分散テスト。
8. OIDN backend(`UMBREON_WITH_OIDN`) + フォールバック warning + ビルドガードテスト + docs(本ファイル追補)。

---

## 検証(end-to-end)

1. `task build && task test` — 全 ctest green。既存 AO/shadow/transparency 回帰が無改変通過(bit-exact)。
2. 視覚確認: `task render -- --gi on --gi-samples 64 --gi-accuracy 0.15 --ao-multiscale on --ao-bent-normal on --denoiser atrous --supersample 3` を `data/1ab0_scene6_densurf.pov`(density surface)に対し、ポケット内の柔らかな indirect・接触影・color bleeding を確認。
3. leak 検証: 薄い仕切りシーンで `--gi-component-reject on/off` を比較、別 component 漏れが抑制されること。
4. denoise 比較: `--denoiser none|atrous|oidn`(OIDN はビルド時)を同一 spp で比較し、エッジ保持とノイズ低減を確認。`--gi-samples` を下げても denoise でノイズが収まること。
5. bit-exact 回帰: `--gi off --denoiser none` の出力が改修前と byte 一致(ハッシュ/PPM compare)。
6. 決定性: GI 全 on の同一シーンを `--threads` 変えて2回、出力 bit 一致。

---

## 参考文献

- Ward & Heckbert, *Irradiance Gradients*(1992) / Jarosz dissertation Ch.3(harmonic-mean 半径 Eq.3.8、weight Eq.3.10、勾配補間 Eq.3.15): https://cs.dartmouth.edu/~wjarosz/publications/dissertation/chapter3.pdf
- Křivánek et al., *Making Radiance and Irradiance Caching Practical: Adaptive Caching and Neighbor Clamping*, EGSR 2006(neighbor clamping、full-convergence、leak 対策): https://cgg.mff.cuni.cz/~jaroslav/papers/2006-egsr/2006-egsr-krivanek-rcadapt-fin-electronic.pdf
- Křivánek, *Practical Global Illumination with Irradiance Caching*, SIGGRAPH 2008 course: https://cgg.mff.cuni.cz/~jaroslav/papers/2008-irradiance_caching_class/index.htm
- Dammertz et al., *Edge-Avoiding À-Trous Wavelet Transform*, HPG 2010(B3 kernel、2^i dilation、N=5): https://jo.dreggn.org/home/2010_atrous.pdf
- Schied et al., *Spatiotemporal Variance-Guided Filtering (SVGF)*, HPG 2017(edge-stop Eq.3-5、σ_z=1/σ_n=128/σ_l=4、albedo 復調): https://cg.ivd.kit.edu/publications/2017/svgf/svgf_preprint.pdf
- Intel Open Image Denoise docs(RT filter、cleanAux、aux range、hdr/inputScale): https://www.openimagedenoise.org/documentation.html — GitHub/releases(v2.3、Apache-2.0、build flag): https://github.com/RenderKit/oidn
- Majercik et al., *DDGI*, JCGT 2019(visibility-weighted blending、leak 対策の参考): https://www.jcgt.org/published/0008/02/01/paper-lowres.pdf
- Embree 4 API(`rtcPointQuery`、incoherent 単発 ray、coherent/incoherent flag): https://github.com/RenderKit/embree/blob/master/doc/src/api.md
