# umbreon Adaptive Surface Irradiance Cache + OIDN 実装プラン

> **SUPERSEDED (2026-07-16)**: 本プランは [../pt2_survey.md](../pt2_survey.md) により
> 置き換えられた。irradiance cache 系は追わない(pt1 が `--gi on` の既定になり、
> cache integrator は比較用に凍結)。§「設計の大前提」のエネルギー・遮蔽の決定
> (Route A、合成式、二重計上ガード)は pt1/pt2 に引き継がれており、そこだけが
> 本書の生きている部分である。

## Context

umbreon に **diffuse GI**(既定 one-bounce、拡張で multi-bounce)を入れる。全ピクセルで GI ray を飛ばすのではなく、表面上の少数代表点(cache record)でのみ irradiance を計算して cache し、shading 時に近傍 record を補間再利用する **irradiance caching**(Ward–Heckbert 1988/1992, Křivánek 2006)。これは **POV-Ray radiosity と同一の手法系**(Greg Ward 型 irradiance cache)であり、本プランの狙いは「POV-Ray radiosity に立体感・速度で勝る」こと。残留ノイズは **denoiser**(自前 edge-aware à-trous を既定、OIDN を optional backend)で処理する。

### 設計の大前提(本プラン最重要・最初に確定する)

**最終合成式を最初に固定し、遮蔽(occlusion)を二重・三重に計上しない。**

irradiance cache の `E_cached` は hemisphere gather で間接光を **遮蔽込みの本物として** 計算した量である。したがって遮蔽は `E_cached` の内部に既に含まれる。これに対し AO は「間接光の遮蔽の近似」であり、**両者を掛け合わせる/足し合わせると同じ遮蔽を多重計上し、AO で作った陰影を indirect が埋め戻すか、逆に過剰に暗化する**。実測症状「白っぽく平坦な画像」は、(遮蔽付き indirect)と(遮蔽なし定数 ambient)を加算し、その和に AO を乗じる旧式が原因だった。

採れる路線は2つで、排他である:

- **A路線(本プランが採用): GI を正とし、AO を最終合成から外す。** POV-Ray radiosity と同じ土俵(遮蔽付き物理 indirect)で、サンプル品質・バウンス数・補間品質を上回ることで「勝る」を狙う。最終合成に定数 ambient 項も AO 乗算も置かない。
- B路線(本プランは採らない): AO を立体感の主役にし、indirect は遮蔽を持たない color bleeding のみに格下げ。近似(AO)が本物(GI)に勝つことは原理的に無いため、「POV-Ray に勝る」目標とは整合しない。

→ **本プランは A路線。** environment fill(sky/ground)は「[E] で定数加算」ではなく、**[C] の gather でレイが miss したときの `environmentRadiance(wi)`** として与え、`E_cached` に溶け込ませる(遮蔽が効く)。bent normal はその gather 方向・env lookup にのみ用い、最終合成 [E] では用いない。contact/shape AO は最終合成に用いず、デバッグ AOV として残すに留める。

### 正規化(これも最初に確定)

cosine-weighted hemisphere gather のモンテカルロ推定では、cosine weight が幾何項 `cosθ` を相殺し、pdf に既に `1/π` が含まれる。よって record の格納値は

```
E_i ≈ (1/N) Σ_s  albedo(y_s) · L_direct(y_s)        // y_s = wi 方向の最近接 hit 点。miss なら environmentRadiance(wi)
```

で、最終の出射 radiance は

```
L_indirect = (albedo/π) · E_cached    と等価。実装上は giIntensity を物理係数 1.0 として:
L = L_direct + giIntensity · kd · E_cached     // kd = mat.diffuse、giIntensity 既定 1.0(純粋にユーザ gain)
```

**旧プランの「POV 非 1/π 慣習として 1/π を giIntensity に吸収する」方針は採らない**(明るさ過剰・正規化曖昧の温床)。まず `giIntensity=1.0`・`1/π` 混入なしで物理的に正しい絵を出し、その後ユーザの好みで gain を調整する。

**前提(土台)**: [ao-quality.md](ao-quality.md) の **AOV/first-hit infra** をそのまま使う。本プランが流用するのは AO の *数値* ではなく *G-buffer 経路* である:
- first-hit / G-buffer AOV(`position`(本プランで追加)/`normal`/`albedo`/`depth`/`objectId`)を主経路で出力するゲート(ao-quality.md フェーズ5)。
- 2色 sky/ground グラデーション(`bent` 由来の方向)を **`environmentRadiance(wi)` の定義**として使う(本プランの [C] gather 内 env fill)。
→ ao-quality.md フェーズ5(AOV 主経路化)+ env グラデーション定義を本プランの**フェーズ0(前提)**とする。`computeAOQuality` の contact/shape/openness は最終合成では使わない(AOV/デバッグのみ)。

> **前提の実装状況(2026-06)**: ao-quality.md は全フェーズ実装済み(`feat/ao-quality`)。`aoWriteAov` ゲートの first-hit/G-buffer AOV(`albedo`/`normal`/`bentNormal` ほか)主経路出力、bent 由来 2 色 sky/ground 方向性 environment は利用可能。本プランは `position` AOV の追加と `IrradianceRecord`/cache 本体/denoiser から着手できる。**注意: contact/shape AO を最終合成へ配線していた旧 [E] は本改訂で撤去する。**

**実体環境**(grounding 確定):
- Intel Embree 4.4.1 + oneTBB 直叩きの自前 C++17 レンダラ。依存は Embree + TBB のみ(`CMakeLists.txt:53-103`)。
- **1シーン1フレーム**・temporal/animation 状態なし(`render()` は毎回 BVH を再構築、`embree_renderer.cpp:204-205`、`embree_renderer.hpp:17-25`)。→ 時間的再利用は不要、cache は単フレーム使い切り。
- **決定性契約**: サンプリングは hi-res `(px,py,sample)` 由来の `tea2` のみ。TBB スレッド数非依存・run-to-run bit 一致を**テストで固定**(`test_render.cpp:858-876`、raw `!=` 比較)。
- レンダ後も `device_`/`scene_` を生かす設計(post-pass の BVH query 用、`embree_renderer.cpp:362-366`)→ cache fill / interpolation のレイキャストはこの live BVH を使える。
- HDR/EXR writer は無い(PNG sRGB + PPM のみ、`image_io.cpp:148-151`)。color buffer は linear HDR、`writeImage` が sRGB encode。

**Scope**: cache + GI は **mesh hit のみ**(AO と同じゲート)。outline/VdW primitive(sphere/cylinder)は flat silhouette で GI ライティング対象外(`hit_shader.hpp:144-146` の `aoFactor=1.0f` リテラルゲートを踏襲)。

**目標**: 物理的に厳密な full path tracing ではなく、「radiosity 近似の低周波 indirect を POV-Ray より高品質・高速に」。POV-faithful な非物理ヘリテージにはユーザ gain(`giIntensity`)で寄せるが、**遮蔽は常に gather 由来の本物**で、定数 ambient と AO 乗算による埋め戻し・多重計上は行わない。

実装状況: **提案・未着手**(旧 one-bounce + AO 合成版は実装後ロールバック済み。本改訂で合成式を A路線へ是正)。

---

## 全体アーキテクチャ(5パス + denoise)

```
[A] Primary G-buffer pass     first hit → position/normal/albedo/depth/objectId(=group)/materialId
                              + L_direct(既存の direct diffuse/spec + shadows)
        │
[B] Record placement pass     決定的に cache record を表面へ配置(seed + neighbor clamp)
        │  (single-thread, canonical order)
[C] Record fill pass          各 record で hemisphere gather → one-bounce(既定) / multi-bounce(拡張)
        │  (TBB 並列, per-record 固定seed)  → E_i, R_i, ∇E_i, bentNormal_i
        │
[D] Interpolation pass        各 shading point で近傍 record を gather・leak 除去・補間 → E_cached(x)
        │  (TBB 並列, read-only)
[E] 最終合成 (A路線)          L = L_direct + giIntensity·kd·E_cached
        │                     ※ 定数 ambient 加算なし / AO 乗算なし(遮蔽は E_cached 内)
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
2. **[C] Fill = 並列・record 独立**。`E_i` 等を TBB 並列計算。**各 record は自分のインデックス/座標 hash から固定 RNG seed**(共有 atomic カウンタ不可)。one-bounce 既定では fill 中に record 間参照なし → データ競合なし・bit 一致。**multi-bounce 拡張(下記)では前 bounce の cache を read-only 参照する**ため、fill を bounce ごとの段に分け(各段は read-only 前段・write 当段)、段内 record 独立を保てば決定性は維持される。neighbor clamping は `min` 演算で順序非依存(commutative)なので各 fill 段の後にまとめて適用可。
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
  Vec3  irradiance;    // RGB indirect irradiance E_i(遮蔽込み。最終合成で AO を掛けない)
  Vec3  bentNormal;    // 平均非遮蔽方向([C] の env lookup 方向。最終合成では未使用)
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
int   giBounces       = 1;       // 1 => one-bounce。>1 => multi-bounce(cache 間接参照、下記)
float giMaxDistance   = 0.0f;    // 0 => scene 依存自動(diagonal*係数)。indirect ray の tfar
float giIntensity     = 1.0f;    // indirect の物理係数=1.0。値はユーザ gain(1/π 混入はしない)
float giAccuracy      = 0.15f;   // 補間精度 a。w_i>1/a で採用、max 有効半径=a·R_i
float giRecordSpacing = 0.0f;    // 0 => 自動。voxel seeding の target 世界間隔
bool  giGradients     = false;   // 並進/回転勾配で線形補間(滑らか・record減)
bool  giAdaptive      = false;   // R_i 小領域の voxel を1段細分し record 追加(凹部精度)
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

各 record で diffuse irradiance を hemisphere gather(`computeAO` と同じ origin offset / self-intersection eps / `cosineSampleHemisphere`、ただし `rtcOccluded1`→`rtcIntersect1` で最近接 hit を取得):
```
seed = hash(recordIndex)              // per-record 固定。共有カウンタ不可
f = frameFromNormal(record.normal)
E = 0 ; bentAccum = 0 ; invDistSum = 0 ; nHit = 0
for s in 0..giSamples-1:
    (u1,u2) = LD or tea2(seed, s)
    wi = cosineSampleHemisphere(u1,u2,f)         // cosine weight が cosθ を相殺(1/π は pdf 由来)
    hit = intersectNearest(O, wi, eps, giMaxDistance)
    if hit:
        y = hit.position
        E += albedo(y) * incomingAt(y, bounce)   // one-bounce: directLightingAt(y)
                                                 // multi-bounce: directLightingAt(y) + 前段 cache lookup
        invDistSum += 1/hit.dist ; nHit++
        // bentNormal は「開いた方向」なので遮蔽 hit は寄与させない
    else:
        E += environmentRadiance(wi)             // = sky/ground グラデーション(ao-quality.md の env 定義)
        bentAccum += wi
record.irradiance = E / giSamples               // = E_i。遮蔽込み。最終合成で AO を掛けない
record.bentNormal = safeNormalize(bentAccum, record.normal)   // [C] env lookup 用。[E] では未使用
record.radius     = clamp(nHit>0 ? nHit/invDistSum : giMaxDistance, Rmin, Rmax)  // harmonic mean
// giGradients: gather 中に並進/回転勾配(Ward–Heckbert/Křivánek)を同時蓄積
```
- `directLightingAt(y)`: y の法線・material で既存 light loop を1回評価(shadow 込み可)。これが1バウンス分の入射。
- **harmonic mean 半径** `R_i = N_hit / Σ(1/r_k)`(近接面が支配 → 凹部で record 密)。`Rmin/Rmax` で clamp(過小爆発・過大外挿を防止)。

#### multi-bounce(`giBounces > 1`、POV 超えの主軸)

POV-Ray radiosity は `recursion_limit` で多段 indirect を持つ。低周波 GI の「回り込み・柔らかさ」はバウンス数で効くため、ここが POV 超えの最有力。安価に実現する:
```
// fill を bounce 段に分割。各段は前段 cache を read-only 参照(決定性維持)。
cache^(0): environment のみ(直接光のない暗所も env で下駄)
for b in 1..giBounces:
    for each record i (TBB 並列, per-record seed):
        gather と同様に wi を飛ばし、hit 点 y で
        incomingAt(y,b) = directLightingAt(y) + interpolate(cache^(b-1), y, n_y, comp_y)
        E_i^(b) = mean over samples
    neighbor clamp を段末に適用
最終 record.irradiance = E_i^(giBounces)
```
- 段ごとに前段 cache を **不変参照**するだけなので、各段内は record 独立 → bit 一致を保てる。
- 多くのシーンで `giBounces=2` が立体感/コストの最良点(3 以上は逓減)。既定は 1(byte-identical 維持)、視覚目標シーンで 2 を推奨。

fill(各段)後、**neighbor clamping**(Křivánek 2006、leak 防止の要)を順序非依存に適用:
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
E_cached = wSum>0 ? Esum/wSum : fallback(environment only)
```
- 信頼 record が無い点(`wSum==0`)は environment radiance のみへフォールバック(穴を作らない。**定数 ambient ではなく env 由来**)。決定的(placement で被覆を担保するが保険)。
- 任意で短い visibility check(x↔record の `rtcOccluded1`)を leak 棄却に追加可(コスト増、既定 OFF)。

### [E] 最終合成(A路線・遮蔽の多重計上を排除)

`shadeLocal` の旧 ambient 項を **indirect 項に置換**する(GI on のときのみ。off は既存式で byte-identical):
```
L = L_direct + giIntensity * kd * E_cached        // kd = mat.diffuse、giIntensity 既定 1.0
```
- **定数 ambient(`mat.ambient * ambLightDir`)を加算しない。** environment は [C] の gather で `E_cached` に既に溶け込んでおり、遮蔽が効く。定数加算は谷を埋め戻し「白っぽく平坦」を生むため撤去。
- **contact/shape AO を乗算しない(`* shapeAO`・`*= contactAO` を撤去)。** 遮蔽は `E_cached` 内に本物として存在。AO 乗算は同じ遮蔽の二重・三重計上で、近似が本物を上書きする。`computeAOQuality` の出力は AOV/デバッグのみに残す。
- **bent normal は [E] では使わない**([C] の env lookup 方向専用)。
- 正規化は Context 記載どおり `giIntensity=1.0`・`1/π` 混入なしで物理整合。明るさはユーザ gain として `giIntensity` で調整。
- `indirect`(= `giIntensity * kd * E_cached`)を FrameResult.indirect に保存(denoise demodulation / debug)。

> **旧式(撤去)**: `indirect = (giIntensity*kd*E_cached + mat.ambient*ambLightDir) * C; L = L_direct + indirect*shapeAO; L *= contactAO` — 遮蔽付き indirect と遮蔽なし定数 ambient の加算 + AO 乗算で多重計上。これが「白っぽく平坦」の直接原因。本改訂で上記 A路線式へ置換。

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

## POV-Ray radiosity に「勝る」ための差別化(設計の核)

合成式を A路線に是正すれば「POV-Ray 並み」までは到達する。「勝る」ための上積みは本プラン内に種があり、優先順は:

1. **multi-bounce**([C] の `giBounces`)。POV-Ray は `recursion_limit` で多段。cache 間接参照で安価に 2 バウンス目を入れると、低周波の回り込み・柔らかさで上回れる。**最優先。**
2. **gradient 補間**([C]/[D] の `giGradients`、Ward–Heckbert 並進/回転勾配)。同じ record 数で補間が滑らかになり、POV-Ray より破綻が少ない=「立体感 + 速度」の速度側で勝てる。
3. **adaptive 細分**(`giAdaptive`、object-space)。`R_i` の小さい高ディテール領域だけ record を増やす。均一 voxel/均一 spacing の素朴設定より凹部の精度で勝てる。

いずれも「遮蔽付き本物 indirect」を前提とする改善であり、AO 近似を最終合成に混ぜない A路線だからこそ効く。

---

## 変更ファイル(中核)

- **新規** `src/umbreon/render/irradiance_cache.hpp`(detail, 非公開) — `IrradianceRecord`/`IrradianceCache`/hash-grid、`buildIrradianceCache(...)`([B]+[C]+multi-bounce 段+neighbor clamp)、`interpolateIrradiance(x,n,comp,geom)`([D])。
- **新規** `src/umbreon/render/denoise.hpp` / `denoise_atrous.cpp` — `DenoiserBackend`、`denoiseAtrous(...)`(自前)、`denoiseOidn(...)`(`#ifdef UMBREON_HAVE_OIDN`)。bench でなく `umbreon` lib 側(`bench_core` は依存なし契約)。
- `src/umbreon/render/secondary_rays.hpp` — fill 用 `gatherIrradiance(...)`(`computeAO` 隣、`rtcIntersect1` 版)。`computeAOQuality` は ao-quality.md(本プランは最終合成で呼ばない)。
- `src/umbreon/render/shading.hpp`(行47-55) — ambient 項を indirect 項へ**置換**(GI on 経路)。**定数 ambient 加算と AO 乗算を撤去。** off は既存式維持。
- `src/umbreon/render/hit_shader.hpp`(行85-92) — `position` first-hit 出力、GI on 時に `interpolateIrradiance` を呼ぶ配線。AO 乗算は最終合成に残さない。
- `src/umbreon/render/transparency.hpp`(行118-126) — first-hit `position` 保存。
- `src/umbreon/umbreon.cpp`(行96-172) — post-pass 順序に [B][C(multi-bounce 段)][D] cache build → [F] denoise を挿入(fog/edge と downsample の間)。`EmbreeRenderer` の live BVH を渡す。
- `src/umbreon/render/render_types.hpp` — `RenderOptions`/`FrameResult` 上記追加(`giBounces`/`giAdaptive` 含む)。
- `CMakeLists.txt`(Embree/TBB block 行50-103 隣) — `option(UMBREON_WITH_OIDN OFF)` + `find_package(OpenImageDenoise CONFIG)` + `UMBREON_HAVE_OIDN` guard + `umbreon` への optional link(行131)。`bench_core` には足さない。
- `src/bench/cli.{hpp,cpp}` / `src/bench/main.cpp` — CLI フラグ(`parseBool`/`parseHexColor` 再利用)、`--dump-aov` に position/indirect 追加。
- `tests/test_render.cpp`(AO 回帰 行754-876 流儀) — 新テスト群。

---

## bit-exact / 決定性 戦略

1. **master gate `gi==false`** で全パス無効 → color byte-identical(既存全テスト無改変通過)。`denoiser==None` 既定。
2. **placement 順序非依存**(voxel 占有集合)+ **fill per-record 固定 seed** + **multi-bounce は段ごと read-only 前段参照**(段内 record 独立)+ **neighbor clamp は min**(可換) + **interpolation read-only** → TBB スレッド数非依存・run-to-run bit 一致。AO の決定性テスト(`test_render.cpp:858-876`)と同形の cache 決定性テストで固定。
3. GI on でも **direct 項は不変**(indirect は加算項。定数 ambient も AO 乗算も無い)。off↔on の差分は indirect 項のみ。
4. **outline ゲート維持**: GI/AO は mesh 経路限定。outline 分岐は `aoFactor=1` リテラル・indirect 非加算で従来一致。
5. denoise は color のみ後段で書き換え、`denoiser==None` で no-op → 既定 byte-identical。OIDN 不在ビルドでも `denoiser==OIDN` 指定時は warning + AtrousBilateral フォールバック(ビルド差で出力が壊れない)。

---

## テスト計画(tests/test_render.cpp、5x5 ortho 流儀)

- **既定 bit-exact**: `gi=false`/`denoiser=None` で全既存ロック値一致(`approx 1e-6`)。
- **GI gate 決定性**: `gi=true` 同一シーン2回 → color raw `!=` で bit 一致(AO 流儀)。multi-bounce(`giBounces=2`)でも同様に bit 一致。
- **遮蔽の単一計上(回帰の要)**: 凹部(谷)を持つシーンで、`gi=true` の谷が `L_direct` のみより**暗く**なる(indirect 由来の遮蔽が出る)一方、白っぽく**平坦化しない**ことを、谷とフラット面のコントラスト不等式で assert。旧式の「定数 ambient + AO 乗算」回帰を防ぐ番人テスト。
- **indirect の方向性(color bleeding)**: 白床+赤壁の近接で、床の壁側 channel が反対側より明るいを不等式 assert。
- **multi-bounce 単調性**: `giBounces=2` の暗所(直接光の届かない凹)輝度が `giBounces=1` 以上(回り込みで明るくなる)を不等式 assert。
- **leak 棄却**: 薄い仕切りを挟む2 component で、`giComponentReject=on` のとき片側の明色が反対側へ漏れない(off 比で漏れ減)を不等式 assert。
- **半径 clamp/neighbor clamp**: 単一被覆 record の有効範囲が `a·R_i` を超えないこと、隣接 clamp 後に過大 `R_i` が縮むこと。
- **denoise no-op**: `denoiser=None` で color 不変。`AtrousBilateral` でノイズ分散が低下しエッジ(深度/法線跨ぎ)が保たれる(平坦領域分散↓、エッジ近傍の片側平均が混ざらない)を assert。
- **OIDN(ビルド時のみ)**: `UMBREON_HAVE_OIDN` ガード下で実行、未定義時はテスト skip + フォールバック warning を確認。

---

## CLI フラグ追加

`--gi on|off`, `--gi-samples <n>`, `--gi-bounces <n>`, `--gi-max-dist <world>`, `--gi-intensity <f>`, `--gi-accuracy <a>`, `--gi-spacing <world>`, `--gi-gradients on|off`, `--gi-adaptive on|off`, `--gi-normal-reject <cos>`, `--gi-component-reject on|off`, `--gi-seed-per-vertex on|off`, `--denoiser none|atrous|oidn`, `--denoise-iters <n>`, `--denoise-sigma-z|n|l <f>`, `--denoise-demodulate on|off`, `--oidn-clean-aux on|off`。`printUsage` と `main.cpp` の `ropt.* = opt.*` 転写に追記。

---

## ビルド(OIDN を optional 依存に)

- `CMakeLists.txt` に `option(UMBREON_WITH_OIDN "Enable Intel OIDN denoiser backend" OFF)`。ON 時のみ `find_package(OpenImageDenoise 2 CONFIG)`、見つかれば `target_compile_definitions(umbreon PRIVATE UMBREON_HAVE_OIDN)` + `target_link_libraries(umbreon PUBLIC OpenImageDenoise)`。
- OIDN 2.3.x / Apache-2.0 / CPU device 既定対応。依存は oneTBB(既存) + oneDNN。RT weights 同梱で lib は中量級 → **optional 必須**。`OIDN_FILTER_RT` のみ有効化で縮小可。
- 既定 OFF で従来通り Embree+TBB のみ。AtrousBilateral は依存ゼロで常時利用可 → OIDN 無しでも denoise 機能は成立。

---

## 実装順(各コミット独立 green、毎回 bit-exact 担保)

0. **前提**: ao-quality.md フェーズ5(first-hit/AOV 主経路化)+ env(sky/ground)グラデーション定義を先行。本プランの土台。**注意: AO は最終合成に配線しない。**
1. `position` first-hit AOV 追加 + FrameResult 拡張(未配線・byte 一致)。
2. `irradiance_cache.hpp`: 構造体 + hash-grid + 決定的 placement([B])。record 集合の決定性テスト(seed 再現)。
3. `gatherIrradiance` + parallel fill([C] one-bounce) + harmonic-mean 半径 + neighbor clamp。fill 決定性テスト(per-record seed, bit 一致)。
4. interpolation([D]) + leak 棄却 + `shadeLocal` indirect **置換**配線([E]、定数 ambient/AO 乗算なし) + `--gi` gate/CLI。**遮蔽の単一計上テスト**・color bleeding・leak・決定性テスト。視覚確認(白っぽく平坦が出ないこと)。
5. multi-bounce([C] の `giBounces`、段ごと read-only 前段参照) — 暗所単調性・決定性テスト。POV 比較の主軸。
6. 勾配補間([C]/[D] の `giGradients`) + adaptive 細分(`giAdaptive`) — 滑らかさ/record 削減/凹部精度の quality 検証。
7. **[完了 e426ba4]** denoise: AtrousBilateral([F] 自前) + CLI + no-op/分散テスト。
   - CLI 既定: `--denoiser` 未指定時は `--gi` on で atrous、off で None(library API 既定は None のまま byte-identical)。
   - テスト: 平坦半面の分散低下 / 法線エッジ保存 / 背景画素不変 / iters=0 no-op。
8. **[完了 e426ba4]** OIDN backend(`UMBREON_WITH_OIDN`) + フォールバック warning + ビルドガードテスト + docs(本ファイル追補)。
   - deplibs oidn-2.5.0 を `find_package(OpenImageDenoise 2 CONFIG)` で検出、`UMBREON_HAVE_OIDN` ガード。未検出 ON ビルドは warning + a-trous フォールバック。
   - Taskfile static 経路で `CMAKE_PREFIX_PATH` に OIDN prefix 追加 + `UMBREON_WITH_OIDN=ON`。
   - 注: 現状の評価条件(gi-accuracy 0.15 / gi-samples 64)で OIDN はほぼ無変化(設定要調整)。残留の暗点は stochastic でなく cache level の構造的アーティファクトのため、両 denoiser とも温存する(別途 cache level の課題)。

---

## 検証(end-to-end)

1. `task build && task test` — 全 ctest green。既存 AO/shadow/transparency 回帰が無改変通過(bit-exact)。
2. 視覚確認: `task render -- --gi on --gi-samples 64 --gi-bounces 2 --gi-accuracy 0.15 --gi-gradients on --denoiser atrous --supersample 3` を `data/1ab0_scene6_densurf.pov`(density surface)に対し、ポケット内の柔らかな indirect・接触影・color bleeding を確認。**白っぽい平坦化が無いこと**を旧式画像と対比。
3. POV-Ray 比較: 同シーンの POV-Ray radiosity 出力と並べ、(a)凹部の立体感(b)color bleeding(c)レンダ時間を比較。multi-bounce/gradient で立体感・速度のいずれかが上回ることを確認。
4. leak 検証: 薄い仕切りシーンで `--gi-component-reject on/off` を比較、別 component 漏れが抑制されること。
5. denoise 比較: `--denoiser none|atrous|oidn`(OIDN はビルド時)を同一 spp で比較し、エッジ保持とノイズ低減を確認。`--gi-samples` を下げても denoise でノイズが収まること。
6. bit-exact 回帰: `--gi off --denoiser none` の出力が改修前と byte 一致(ハッシュ/PPM compare)。
7. 決定性: GI 全 on(multi-bounce 含む)の同一シーンを `--threads` 変えて2回、出力 bit 一致。

---

## 参考文献

- Ward & Heckbert, *Irradiance Gradients*(1992) / Jarosz dissertation Ch.3(harmonic-mean 半径 Eq.3.8、weight Eq.3.10、勾配補間 Eq.3.15): https://cs.dartmouth.edu/~wjarosz/publications/dissertation/chapter3.pdf
- Křivánek et al., *Making Radiance and Irradiance Caching Practical: Adaptive Caching and Neighbor Clamping*, EGSR 2006(neighbor clamping、full-convergence、leak 対策): https://cgg.mff.cuni.cz/~jaroslav/papers/2006-egsr/2006-egsr-krivanek-rcadapt-fin-electronic.pdf
- Křivánek, *Practical Global Illumination with Irradiance Caching*, SIGGRAPH 2008 course: https://cgg.mff.cuni.cz/~jaroslav/papers/2008-irradiance_caching_class/index.htm
- POV-Ray documentation, *Radiosity*(`recursion_limit`/`error_bound`/`pretrace`、Greg Ward 型 irradiance cache であることの確認): https://www.povray.org/documentation/view/3.7.0/350/
- Dammertz et al., *Edge-Avoiding À-Trous Wavelet Transform*, HPG 2010(B3 kernel、2^i dilation、N=5): https://jo.dreggn.org/home/2010_atrous.pdf
- Schied et al., *Spatiotemporal Variance-Guided Filtering (SVGF)*, HPG 2017(edge-stop Eq.3-5、σ_z=1/σ_n=128/σ_l=4、albedo 復調): https://cg.ivd.kit.edu/publications/2017/svgf/svgf_preprint.pdf
- Intel Open Image Denoise docs(RT filter、cleanAux、aux range、hdr/inputScale): https://www.openimagedenoise.org/documentation.html — GitHub/releases(v2.3、Apache-2.0、build flag): https://github.com/RenderKit/oidn
- Majercik et al., *DDGI*, JCGT 2019(visibility-weighted blending、leak 対策の参考): https://www.jcgt.org/published/0008/02/01/paper-lowres.pdf
- Embree 4 API(`rtcPointQuery`、incoherent 単発 ray、coherent/incoherent flag): https://github.com/RenderKit/embree/blob/master/doc/src/api.md
