# umbreon AO 立体感向上 実装プラン

## Context

umbreon のAOは「かかり方が弱く平坦」。計算・メモリコストを犠牲にしても質を上げたい(ただし GI はスコープ外)。

**前提の訂正**: umbreon は OSPRay を使っていない。実体は **Intel Embree 4.4.1 + oneTBB 直叩きの自前 C++17 レンダラ**で、AO は「OSPRay scivis/ao のアルゴリズムを C++17 へ移植した自前実装」(`secondary_rays.hpp`)。依存は Embree + TBB のみ(OIDN 等のデノイザ依存なし)。改修はこの自前パイプライン内で完結する。

**「弱く平坦」の2つの根本原因**(調査で確定):
1. **binary occlusion + 単一大半径**(`aoDistance ≈ diagonal*0.7`)。接触影が出ず、遠近で寄与が一定。`secondary_rays.hpp:103` `computeAO` は `rtcOccluded1` の二値判定のみ。
2. **AOが ambient 項のみを変調**。`shading.hpp:53-55`: `out = emission*C + aoFactor*mat.ambient*C*ambLight + Σlights(...)`。direct diffuse はAO非依存なので、`mat.ambient` が小さいシーンでは品質を上げても画面上の効果が頭打ち。

**目標**: radiosity 完全再現ではなく「radiosity に近く見える形状強調 + 過暗化しない接触陰影」。

**確定方針**:
- diffuse 減衰レバー `aoDiffuseFactor`(既定0)を**用意する**。
- リサーチ提案を**全フェーズ実装**する。
- AOフェーズでは **OIDN を追加しない**(決定論的低食い違いサンプリング + supersample で品質確保)。ただし将来の surface irradiance cache は OIDN を前提とするため、本AOは **OIDN を後から差し込める形**(guide AOV = albedo/normal を主経路で出力)で実装する。
- AOは「完成形」を作り込まず、将来の **surface irradiance cache + OIDN** で**そのまま再利用できる infra** まで整える(下記「将来拡張との互換設計」)。AO単体を radiosity 化する方向(大量サンプル / screen-space GTAO 深追い / AO への color bleeding / AO だけで陰内勾配)は採らない。

実装状況: **実装済み**（全フェーズ 1–5 完了、`feat/ao-quality` ブランチ）。CLI フラグ
`--ao-falloff`/`--ao-multiscale`/`--ao-bent-normal`/`--ao-sky`/`--ao-ground`/`--ao-camera-up`/
`--ao-up`/`--ao-multibounce`/`--ao-ld`/`--ao-diffuse`/`--ao-write-aov` で利用可。`--dump-aov` は
`--edges` 非依存に一般化済み。既定 OFF で bit-exact（既存 AO 回帰を無改変通過）。API は
`docs/api/libumbreon.md` 参照。

## 将来拡張との互換設計(surface irradiance cache + OIDN)

本AOは単体の完成を目指さず、後段の diffuse GI cache + OIDN へ無改修で接続できる基盤を兼ねる。調査結論「AO を作り込みすぎず、最終案で再利用できる形まで」に従い、**再利用する項目のみ**深掘りする。

**既存インフラの再利用**(edge パイプライン用に実装済み):
- `FrameResult` は既に `albedo`/`normal`/`depth` AOV スロットを持つ(`render_types.hpp:175-186`)。`normal`/`viewZ`/`objectId`/`materialId` は first-hit(G-buffer)として edge 経路で書かれている(`embree_renderer.cpp:264-270, 324-331`)。
- `--dump-aov <prefix>` の false-color ダンプ機構が既にある(`main.cpp:501-554`、現状 `--edges` 必須)。
- → AO の将来互換は**新規インフラではなく既存の AOV/G-buffer/ダンプ機構の一般化**で済む(実装コスト小)。

**この AO 作業で「将来再利用前提」に整える項目**:
1. **first-hit / G-buffer を主経路で出力**: `albedo`(未配線)と `normal` を AOV ゲートで書く。cache の空間キー・OIDN の guide buffer・edge-aware upsample の guide を兼ねる。
2. **contact AO と shape AO を分離保持**: `AOResult` で小半径(contact)と中半径(shape)を**合成せず別値で返す**。最終合成は `L = L_direct + A_shape*(L_indirect + L_env); L *= A_contact` を想定(cache 投入時は `L_indirect` を cache 値に差し替えるだけ)。AO 単体期は両者をブレンドして従来の単一 `aoFactor` に縮約。
3. **bent normal を AOV として外出し**: 方向性 ambient の内部利用(フェーズ2)に加え、cache lookup 方向 / environment fill / debug 用に出力。
4. **平均ヒット距離 AOV**: 暗さの原因が「近接接触」か「遠方遮蔽」かを切り分ける調整用(調査の指摘)。同一レイセットの `t` から無料で算出。
5. **debug 可視化**: `--dump-aov` を AO 経路でも有効化し contact/shape/occlusion/avgHitDist/bentNormal/normal/depth/albedo/objectId を出力。

**今は実装せず、infra だけ予約する項目**(cache 投入時に追加):
- **OIDN 統合**: 本AOでは guide(albedo/normal)を出すところまで。デノイズ呼び出し自体は cache フェーズ。
- **半解像度 AO + edge-aware(joint-bilateral)upsample**: normal/depth/objectId を guide に。cache は高コストなので半解像度が標準。AO 単体は supersample で足りるため未実装。
- **surface irradiance cache 本体**(diffuse GI)。AO の multibounce / `aoDiffuseFactor` はこの cache が入るまでの**暫定近似**で、cache 投入時に置換される(下記でも明記)。

## 設計の核心(全フェーズ共通)

1. **単一レイセット共有でレイ増ゼロ**: `aoSamples` 本を最大半径 `R=aoDistance` へ `rtcIntersect1` で飛ばし最近接ヒット距離 `t` を取得。マルチスケール/距離減衰/bent normal はこの1セットの `t` と方向から算術で導く。レイ本数は現状維持、per-ray は `rtcOccluded1→rtcIntersect1` で約1.2-1.5x。メモリ増ゼロ。
2. **二重ゲートで bit-exact 既定OFF**:
   - `aoSamples==0`(既存ゲート)で全AO無効=従来とbyte一致。
   - `aoEnhanced()==false`(全 enhancement フラグ既定)のとき**レガシー `computeAO` をそのまま呼ぶ**ので、`--ao-samples` だけ指定した既存ユーザの出力も現状と完全一致(既存AO回帰テストを無改変で通過)。
   - `aoEnhanced() = aoFalloffPower>0 || aoMultiScale || aoBentNormal || aoMultibounce || aoLowDiscrepancy || aoDiffuseFactor>0`。
3. **決定論・スレッド非依存の維持**: `rtcIntersect1` は決定論的最近接ヒット。シードは hi-res `(px,py,sample)` 由来(`tea2`)。TBB スレッド数/grain 非依存、run-to-run bit 一致。新規算術(bent/multiscale/multibounce)も全て決定論的。
4. **公開面に Embree を漏らさない**: 新構造体 `AOParams`/`AOResult` は `secondary_rays.hpp`(内部・Embree込み)に隔離。`RenderOptions`(公開)へは plain な `bool/float/配列` のみ追加。CueMol2/3 は静的リンク・再コンパイルなので既定値で旧挙動維持。

## 変更ファイル(中核)

- `src/umbreon/render/secondary_rays.hpp` — 既存 `computeAO`(行103)は温存。新規 `struct AOParams`、`struct AOResult{ float contact; float shape; float openness; Vec3 bent; float avgHitDist; }`(contact/shape は将来 cache 合成用に**未ブレンド**で保持、`openness` は AO 単体期の縮約値)、`AOResult computeAOQuality(...)`、ヘルパ `aoFalloff`/`aoMultibounce`、(任意)Hammersley + Cranley-Patterson。
- `src/umbreon/render/hit_shader.hpp`(行85-92) — `aoEnhanced()` で enhanced/legacy 分岐。enhanced 時は `computeAOQuality` を呼び `Vec3 aoFactor` と方向性 `ambLight` を組み立てる。outline 分岐(行144)は不変(リテラル渡し)。
- `src/umbreon/render/shading.hpp`(行47-55) — ambient 変調を `float aoFactor` → `Vec3 aoFactor` に拡張(フェーズ3で必須、**フェーズ1で先行導入して churn 回避**)。`aoDiffuseFactor` を light loop の diffuse 蓄積に乗算。`ambLight` は既に Vec3 なので方向性ambientはシグネチャ変更不要。
- `src/umbreon/render/render_types.hpp`(`RenderOptions` 行127) — 新フィールド追加(全て既定=現状再現)。`FrameResult` に AOV スロット追加(`contactAo`/`shapeAo`/`bentNormal`/`avgHitDist`、既存 `albedo`/`normal`/`depth` は流用)。
- `src/umbreon/render/embree_renderer.cpp`(行231/274/264-331) — `ShadeContext` に gradient 用 up 軸を1つ追加。AOV 出力ゲート時に `albedo`/`normal`(主経路、現状 edge 限定)と AO 派生 AOV(contact/shape/bent/avgHitDist)を書く。ゲート OFF 時は edge 経路と同様に空のままで byte-identical。
- `src/bench/main.cpp`(AOV ダンプ 行501-554) — `--dump-aov` を `--edges` 非依存に一般化し、AO 派生 AOV(contact/shape/avgHitDist/bentNormal/albedo)を false-color 出力。
- `src/bench/cli.hpp` / `cli.cpp` / `src/bench/main.cpp`(AOブロック 行398-410) — CLI フラグ追加(`parseHexColor`/`parseBool` 既存を再利用)。
- `tests/test_render.cpp`(AO回帰 行754-876 直後) — 新テスト群。

## RenderOptions 追加フィールド(全て既定=現状再現)

```cpp
// --- AO quality (all default to legacy binary single-scale behavior) ---
float aoFalloffPower = 0.0f;     // 0 = binary (legacy); >0 => (max(0,1-t/R))^power
bool  aoMultiScale   = false;    // false = single radius (aoDistance); true = 3-scale
bool  aoBentNormal   = false;    // directional ambient from average unoccluded dir
float aoSkyColor[3]    = {1,1,1}; // up-hemisphere tint (x scene.ambientColor)
float aoGroundColor[3] = {1,1,1}; // down-hemisphere tint
bool  aoUseCameraUp  = true;     // gradient axis = camera up (view-stable)
float aoUp[3]        = {0,1,0};  // explicit axis when aoUseCameraUp == false
bool  aoMultibounce  = false;    // albedo-aware GTAO cubic (prevents over-darkening)
bool  aoLowDiscrepancy = false;  // Hammersley + per-pixel CP rotation
float aoDiffuseFactor = 0.0f;    // 0 = ambient-only (POV/scivis); >0 also darkens diffuse
```
マルチスケール比率/重みは当面ヘッダ内定数(`radius = {0.08,0.30,1.0}*aoDistance`, `weight = {0.55,0.35,0.10}`)。将来 `aoScaleFrac[3]`/`aoScaleWeight[3]` で露出可。

## フェーズ1 — マルチスケール + 距離減衰 ambient obscurance

`computeAOQuality` 擬似コード(レイ増ゼロ):
```
f = frameFromNormal(Ns); ng = faceForward(Ng→Ns); O = P + ng*eps
R = radius[K-1]                          // = aoDistance
obsSum = 0
for i in 0..N-1:
  (u1,u2) = sample2d(px,py,i)            // TEA(既存) or Hammersley+CP (phase4)
  dir = cosineSampleHemisphere(u1,u2,f)
  if dot(dir,Ns) < 0.01: obsSum += 1; continue     // grazing = 接触扱い
  t = intersectNearest(O, dir, eps, R)  // rtcIntersect1; miss → t=+inf
  obs = 0
  for k in 0..K-1:
    if t < radius[k]: obs += weight[k] * pow(max(0,1-t/radius[k]), aoFalloffPower)
  obsSum += obs
openness = 1 - obsSum/N
```
- これは `Σ_k weight[k]*AO_k` と等価(最近接遮蔽物が支配的というAOの本質に合致)。
- 単一スケール(`aoMultiScale=false`)は K=1, R=aoDistance, weight=1 に退化。`aoFalloffPower=0` は falloff≡1(=binary 相当)。
- `aoDistance` を最大半径 R として再利用(別パラメータ λ/R 不要)。多項式 falloff はコンパクトサポートで tfar=R の不連続なし(指数 falloff より分子表面に自然)。

hit_shader 適用:
```
if opt.aoEnhanced():  r = computeAOQuality(...);  open = r.openness
else:                 open = computeAO(...)            // legacy bit-exact path
```

## フェーズ2 — bent normal + 方向性 ambient

`computeAOQuality` を `AOResult{contact, shape, openness, bent, avgHitDist}` の多値返しに拡張:
```
bentAccum  += (1 - obs) * dir         // 開いた方向ほど寄与大(距離減衰込み)
bent = safeNormalize(bentAccum, Ns)   // 深いポケット(全遮蔽)は Ns へフォールバック
// contact/shape は別スケール集計を畳まずに保持(将来 cache 合成用):
//   contact = 1 - obsSum[小半径]/N,  shape = 1 - obsSum[中・大半径]/N
//   openness(AO単体期の縮約) = 1 - Σ_k weight[k]*obsSum[k]/N
// avgHitDist = ヒットしたサンプルの t 平均(暗さ要因の切り分け用、レイ増ゼロ)
```
hit_shader で方向性 ambient(hemispherical sky/ground gradient):
```
B = aoBentNormal ? r.bent : N
up = aoUseCameraUp ? trueUp(camera) : opt.aoUp
w = 0.5*(dot(B, up) + 1)
ambLightDir = scene.ambientColor ⊙ lerp(aoGroundColor, aoSkyColor, w)   // → shadeLocal の ambLight 引数
```
- HDRI/環境が無いので SH は不採用。2色グラデーションが最小十分。
- 既定 sky==ground==white → `lerp` 定数 → `ambLightDir == scene.ambientColor` で byte 一致。
- up 軸は **camera up 既定**(分子は任意姿勢で世界upが無意味なため、view-stable な頭上ソフトボックス風)。CueMol は `aoUp` で明示上書き可。

## フェーズ3 — albedo-aware multibounce 補正(墨潰れ防止)

GTAO の Jimenez cubic(per-channel、`lerp(A,1,k·lum)` より原理的):
```
multibounce(ao, alb):
  a=2.0404*alb-0.3324; b=-4.7951*alb+0.6417; c=2.7552*alb+0.6903
  return max(ao, ((ao*a+b)*ao+c)*ao)
```
`aoFactor` を Vec3 化して per-channel 適用(pigment C は hit_shader で取得済み):
```
ao_c  = aoMultibounce ? multibounce(openness, C_c) : openness
aoF_c = 1 - aoIntensity*(1 - ao_c)                        // Vec3
```
白(alb≈1)で大きく持ち上げ、暗色で控えめ → 淡色分子表面の凹部過暗化を防ぐ。無効時は `ao_c=openness` で従来同一。

## フェーズ4 — 低食い違いサンプリング(progressive/denoise の代替)

`aoLowDiscrepancy=true` で 2D サンプルを Hammersley(i,N) + per-pixel Cranley-Patterson 回転(オフセットは `tea2(px,py)` 由来=決定論維持)。aoSamples 固定で AO 分散を低減。レイ/メモリ増ゼロ。**AO フェーズでは OIDN/progressive accumulation を追加しない**(supersample box-average が空間デノイザとして機能=実効サンプル `aoSamples × ss²`)。OIDN は将来の cache フェーズで導入するため、その guide AOV(albedo/normal)はフェーズ5で先行整備する。

## フェーズ5 — first-hit / AOV 出力 + debug 可視化(cache/OIDN 互換)

新フラグ `aoWriteAov`(既定 false)で AOV 出力をゲート。OFF 時は edge 経路と同じく空のままで color 出力 byte-identical(AOV はカラーと別バッファなので有効化しても color 演算列は不変)。

主経路で書く AOV:
- **albedo**(`C`、未配線スロットへ)/ **normal**(face-forwarded `N`、現状 edge 限定を一般化)= OIDN guide + cache 空間キー。
- **contactAo / shapeAo / bentNormal / avgHitDist** = `computeAOQuality` の `AOResult` から。cache lookup 方向・調整・将来の合成入力。
- depth は既存(常時)。objectId/materialId は edge AOV を流用(将来 component 別 cache key)。

`--dump-aov <prefix>` を `--edges` 非依存に一般化し、上記を false-color 出力(avgHitDist は半径正規化、bentNormal は `*0.5+0.5`)。これは AO 調整(暗さ要因の切り分け)に直接効き、cache/OIDN デバッグでもそのまま使える。

## 任意レバー — aoDiffuseFactor(暫定の立体感スイッチ、既定0)

根本原因2(ambient 項頭打ち)を打破。`shadeLocal` の light loop の **diffuse 蓄積のみ**に乗算(specular/highlight には掛けない):
```
diffuse_term *= (1 - aoDiffuseFactor*(1 - openness))
```
既定0で bit-exact。POV/scivis の「AOはambientのみ」契約からは逸脱(物理的にはGIの粗近似)だが、分子ビューア(PyMOL等)の標準的手法。CueMol/ユーザが任意に有効化。

**位置づけ(重要)**: `aoDiffuseFactor` と フェーズ3 の `aoMultibounce` は「AO で陰内の明るさを作る」暫定近似であり、調査が「行き過ぎ」と指摘した領域に踏み込むもの。将来の **surface irradiance cache(diffuse GI)が入れば置換される**。実装コストが小さく既定 OFF なので cache 完成までの中間提供として残すが、ここを深掘りして radiosity 化はしない(その労力は cache へ回す)。立体感の本命は AO 単体ではなく cache 側。

## bit-exact / 決定論 戦略(集約)

1. 二重ゲート(`aoSamples==0` と `aoEnhanced()==false`)。後者でレガシー `computeAO` を呼ぶため既存AO出力も完全一致。
2. ambient 項は `aoFactor_c * mat.ambient * C_c * ambLight_c` の乗算のみ。既定 `aoFactor={1,1,1}`, `ambLight=scene.ambientColor` で旧コードと同一float演算列。Vec3化は各ch `{s,s,s}` 同値で先行導入しても bit 一致。
3. 新パスの決定論: `rtcIntersect1` 最近接 + hi-res pixel シード。スレッド数非依存・run-to-run bit 一致。
4. outline/VdW ゲート維持: outline 分岐は `aoFactor={1,1,1}` リテラルで enhanced 経路に絶対入らない。

## テスト計画(tests/test_render.cpp、既存 5x5 ortho イディオムに倣う)

- **既定 bit-exact**: `aoSamples=16` + enhancement 無し → center が既存ロック値と一致(レガシーパス検証、既存テスト流用)。
- **距離減衰の分離**: 遠方スラブで falloff-on(power2) > binary(遠方遮蔽が緩む)、near-contact スラブで falloff-on ≈ binary(接触は暗いまま)。
- **マルチスケール**: 接触のみ vs 接触+遠方(R_large のみ届く)で multiscale_center < smallScaleOnly_center。
- **bent normal 方向性ambient**: sky={1,1,1}/ground={0,0,0}, up=+z で center≈1、up=−z で center≈0。bent が傾く向きで単調 assert。
- **albedo multibounce**: 白床(C=1) vs 灰床(C=0.5)で (occ/open)_white > (occ/open)_gray。
- **aoDiffuseFactor**: 0で現状一致、>0で diffuse 低下を不等式 assert。
- **enhanced 決定論**: 全 enhancement on の同一シーン 2回 → color bit 一致。
- **outline ゲート**: enhanced 全on の flatOutline 球 center が AO-off と厳密一致。
- **AOV ゲート byte-identical**: `aoWriteAov` on/off で **color 出力が完全一致**(AOV 有効化が色を変えないこと)。さらに on で `albedo`/`normal`/`contactAo`/`shapeAo` の各バッファが非空かつ既知の代表ピクセルで妥当値(albedo≈pigment、normal≈面法線、contact≤shape の方向性)。
- **contact/shape 分離**: 近接スラブで contact が顕著に低下し shape は相対的に保たれる(別値で返っていることの検証)。

## CLI フラグ追加(cli.hpp/cpp, main.cpp 行398-410)

`--ao-falloff <power>`, `--ao-multiscale on|off`, `--ao-bent-normal on|off`, `--ao-sky #RRGGBB`, `--ao-ground #RRGGBB`, `--ao-camera-up on|off`, `--ao-up x,y,z`, `--ao-multibounce on|off`, `--ao-ld on|off`, `--ao-diffuse <frac>`, `--ao-write-aov on|off`(フェーズ5、AOV/G-buffer 出力ゲート)。`parseHexColor`/`parseBool` を再利用し `printUsage` と `main.cpp` の `ropt.ao* = opt.*` 転写に追記。`--dump-aov` は既存(フェーズ5で `--edges` 非依存化)。

## 実装順(各コミット独立 green、毎回 bit-exact 担保)

1. `Vec3 aoFactor` 先行導入(shadeLocal/hit_shader/outline literal `{1,1,1}`) — byte 一致確認。
2. `AOParams`/`AOResult` + `computeAOQuality`(falloff+multiscale+bent 併算)追加(未配線・コンパイル証明)。
3. hit_shader で `aoEnhanced()` 分岐 + フェーズ1フィールド + CLI → 距離減衰/マルチスケール test。
4. bent normal + 方向性ambient(ShadeContext に up 追加) + フィールド/CLI → bent test。
5. albedo multibounce → albedo test。
6. 低食い違いサンプリング + aoDiffuseFactor → test。
7. **フェーズ5: first-hit/AOV 出力 + `--dump-aov` 一般化**(`aoWriteAov` ゲート、albedo/normal 主経路化、contact/shape/bent/avgHitDist スロット) → AOV ゲート byte-identical test + contact/shape 分離 test → docs(本ファイル + `ao-soft-shadow.md` に追補)。cache/OIDN 互換の土台はここで完了。

## プリセット指針(立体感の2系統)

「立体感」には2系統あり、シーン種で効くものが違う:
- **(a) 広域の窪み暗化**: 大半径の一様 AO + 凹部の直接 diffuse 減光。**density surface のような
  滑らかな塊で支配的**。レバー: 大半径 AO(falloff/multiscale を使わない)+ `--ao-diffuse` +
  過暗化防止の `--ao-multibounce`。
- **(b) 接触影 + 方向性**: 近接の締まった影と bent normal の方向性。レバー: `--ao-multiscale` /
  `--ao-falloff` / `--ao-bent-normal`。ボール&スティックや密な接触のあるモデルで効く。

注意: `--ao-falloff` と `--ao-multiscale` は**広域 (a) を弱める**(遠方遮蔽を割り引く)。
滑らかな density surface ではこれらを足すと逆に平坦化するため、(a) 目的では使わない。
検証比較は `~/tmp/ao_compare_densurf/`(`comparison_depth.png`)を参照。

**推奨プリセット(density surface、影をしっかり)**: `--ao-multibounce on --ao-diffuse 0.6`
(= legacy の広域 AO を残し凹部を直接 diffuse で深く落とす。過暗化は multibounce が防ぐ)。

## 検証(end-to-end)

1. `task build && task test` — 全 ctest green。特に既存 AO 回帰(test_render.cpp:754-876)が無改変で通過すること(bit-exact 担保)。
2. 視覚確認(影を強く=系統(a)): `task render -- --ao-samples 48 --ao-multibounce on --ao-diffuse 0.6 --supersample 3` を `data/1ab0_scene6_densurf.pov`(density surface、AO検証の定番)に対して実行し、`out.png` で窪み暗化・凹部の彫りの深さを確認。接触影・方向性(系統(b))を見たい場合は `--ao-multiscale on --ao-falloff 2 --ao-bent-normal on --ao-sky #ffffff --ao-ground #808080` を使う(ただし滑らかな表面では暗化は弱まる)。
3. bit-exact 回帰: enhancement 全 OFF + `--ao-samples 16` の出力が改修前と byte 一致(`--compare` PPM or ハッシュ)。
4. 立体感の段階確認: `--ao-diffuse` の値を上げるほど凹部の陰影が強まることを目視。
5. AOV/cache 互換確認: `--ao-write-aov on --dump-aov out_aov` で `out_aov_{albedo,normal,contactAo,shapeAo,avgHitDist,bentNormal}.png` を出力し、albedo=pigment・normal=面向き・contact が溝で暗く・avgHitDist が「近接接触(暗・小距離)」と「遠方遮蔽(暗・大距離)」を弁別できることを目視。これらが将来の cache lookup / OIDN guide / edge-aware upsample の入力になる。
5. AOV/cache 互換確認: `--ao-write-aov on --dump-aov out_aov` で `out_aov_{albedo,normal,contactAo,shapeAo,avgHitDist,bentNormal}.png` を出力し、albedo=pigment・normal=面向き・contact が溝で暗く・avgHitDist が「近接接触(暗・小距離)」と「遠方遮蔽(暗・大距離)」を弁別できることを目視。これらが将来の cache lookup / OIDN guide / edge-aware upsample の入力になる。

---

## 追補 (2026-06-30): GTAO 相当の実測レシピ + 実プリミティブへの AO 拡張

`ao_test1.pov`(実 SES 表面)で OpenGL GTAO 参照(`ao_test1_ogl.png`)と並べた検証で確定した点。
詳細なリファレンスは `docs/api/libumbreon.md`(§4.6) と `docs/umbreon_cli.md`(奥行き節)。

### 実測レシピ(GTAO 同等の奥行き)

```
--ao-samples 256 --ao-multiscale on --ao-bent-normal on \
--ao-falloff 0 --ao-diffuse 1.0 --ao-distance 40   # 600^2, scene 径 ~65
```

効くレバーの順:
1. **`aoDiffuseFactor=1.0`** — 本命。direct diffuse を凹部で減光して初めて強い陰影が出る。
   既定(`aoDiffuseFactor=0`)では AO は ambient 項のみ減光 → AO OFF と全体の ~2% しか変わらず
   「陰影が弱い」状態だった(計測: AO on vs off の平均|差| 5.2/255)。
2. **`aoFalloffPower=0`(binary)** — 谷を最も鋭く落とす。`>0` は遠方遮蔽を緩めコントラスト低下。
3. **`aoDistance` は大きめ(シーン径 0.5〜0.85 倍)** — このスケールでは**大半径ほど強い**。
   短半径(d6)は近傍に occluder が無く AO OFF と同等(直感に反する)。

注意: ambient のエネルギー配分(`_amb_frac`)引き上げは AO の効きを上げない(GI 専用に回り direct が
減るだけ → 全体が暗くなりコントラストはむしろ低下、計測 D 系列)。強度は `aoDiffuseFactor` で上げる。

### 実プリミティブへの AO 拡張(commit 0acb107)

旧実装は AO をメッシュヒットのみに適用し、球・シリンダは「flat outline」として一律除外していた
ため、ポケット内の CPK 原子球が暗くならなかった。**実 CSG プリミティブ(VdW 原子球・bond
シリンダ; `fromEdgeMacro=false`)にもメッシュ同様 AO を適用**するよう変更。NPR outline 装飾
(`fromEdgeMacro=true`)はフラットのまま。区別は由来タグを `BuiltScene` に伝播して行う
(`sphereFromEdge`/`cylFromEdge`/`cylCapFromEdge`)。AO factor 計算は `computeAoShade()` に集約し
mesh/primitive 両分岐で共用(mesh は bit-exact)。リガンド領域の計測: 平均輝度 136.2 → 90.8。
