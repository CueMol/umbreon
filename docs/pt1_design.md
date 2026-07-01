# pt1 設計メモ: パストレース間接光インテグレータ

`docs/plans/PLAN_pt1_integrator.md` の実装にあたって確定した規約・設計判断の記録。
調査結果と指示書からの適合一覧は `docs/pt1_survey.md` を参照。

## アーキテクチャ: 「間接光ポストパスの代替」

pt1 は独立したレンダラではなく、既存 GI ポストパス
(`EmbreeRenderer::render()` の `opt.gi` ブロック)の irradiance cache を置き換える
**もう一つの間接光インテグレータ**である(`--integrator pt1` / `RenderOptions::giIntegrator`)。

- 直接照明はメインのピクセルループが両インテグレータ共通で計算する。
  指示書 Phase 2 の「直接照明が既存レンダラと一致」は構成上ビット単位で成立する
  (`tests/test_pt1_render.cpp` がロック)。
- cache が record 配置 + gather + 補間で求める間接照度 E を、pt1 はピクセルごとの
  総当たり cosine 半球 gather で直接推定する(cache が近似しているground truth)。
- 合成は cache と同一: `color += giIntensity * giReflectance * E`。
  `giReflectance = kd * pigment`(一次ヒットで取得済み)。

## 放射量の規約(重要)

- バッファに保存する間接照度は **`E_stored = mean(L_i) = E_true / π`**
  (cosine 重み付きサンプルの算術平均。1/π 係数なし)。これは cache の規約
  (`irradiance_cache.hpp` の gradient コメント参照)と同一で、A/B 比較が単位互換になる。
- 指示書 3-1 の推定量 `E_gather ≈ (π/N) Σ L_i` は、本 repo の規約では
  `E_stored = (1/N) Σ L_i` に対応する(合成側に π を掛けないため)。
- 平面テスト(一様 sky = 1)の期待値は指示書の「E = π」ではなく **`E_stored = 1.0`**。
- シェーディングは POV 流の `C * (Ka·amb + Kd·N·L·Lc)`(BRDF の 1/π なし)。
  直接・間接とも同じ流儀なのでエネルギー比は保たれる。

## 二重計上の防止(sky / emission の分担)

1. **sky は gather のミス項のみ**で評価する(`environmentRadiance`)。直接照明パスに
   sky は存在しない。定数 ambient は GI 有効時にメッシュヒットでゼロ化されるので
   (hit_shader の gi ゲート)、ambient と gather sky も二重計上にならない。
   - 例外: env-dome ライト(`--env-light`)は sky を**直接光**として近似する仕組み
     なので、GI と併用すると sky が二重計上される。pt1 は併用時に警告を出す。
     GI の sky は `--sky` / `--sky-radiance` / `--gi-env-intensity` で制御すること。
2. **gather ヒット点に emission は加えない**。光源からシェーディング点への直接光は
   直接照明パスで計上済みであり、gather ヒット点(バウンス点)で自己発光を加えると
   同じエネルギーを二度数えることになる。`oneBounceRadiance` は反射光
   (シャドウテスト済み直接照度 × kd·pigment)のみを返す。

## sky モデル

`environmentRadiance` の sky/ground 2色グラデーションをそのまま使う
(radiance = `envIntensity × ambLight × lerp(ground, sky, 0.5·(dot(ω,up)+1))`)。

- `--sky uniform`(既定): sky = ground = `--sky-radiance`(既定 1,1,1)。
  既定値では cache の環境と完全に一致する(A/B パリティ)。
- `--sky gradient`: 天頂 = `--sky-radiance`、地面 = `--ao-ground`、軸 = AO と同じ
  up 軸(camera-up 既定)。

## cache との意図的な差異

- **gather tfar**: pt1 の既定は ∞(`--gi-max-dist` 指定時はその値)。cache は
  既定で `0.1 × シーン対角`に切り詰める(コントラスト重視の演出的選択)。
  pt1 は ground-truth 参照のため物理既定とする。A/B 画像に差が出る要因として認識する。
- **幾何法線ガード**: pt1 は `dot(ω, N_geom) ≤ 0` のサンプルを寄与 0 で棄却する
  (シェーディング法線と幾何法線の乖離対策、指示書 3-2)。cache には無い。
  full-res モードでは G-buffer に幾何法線が無いためシェーディング法線で代用
  (cosine サンプリングが `dot(ω,N)>0` を保証するので実質無効)。半解像度モードは
  独自 G-buffer に幾何法線を持つため真のガードが効く。
- **AOV**: pt1 は `indirect` と `giOcclusion`(ピクセルの gather ヒット率)を埋める。
  `giRecordViz`(record 密度の可視化)は cache 固有の概念なのでゼロのまま。

## その他の設計判断

- **乱数**: 既存 `tea2` ハッシュ。seed はピクセル index と `--seed` から決定的に生成、
  サンプル index を counter として混合。スレッド間共有なし(TBB スレッド数非依存)。
- **自己交差**: 既存 `selfIntersectEps()`(OSPRay 式スケール適応)。
- **CSG プリミティブ**(原子球・結合円柱): gather レイのヒットは黒のオクルーダ
  (cache と同一、`oneBounceRadiance` が非メッシュで 0 を返す)。将来の改善候補。
- **透明面**: 間接光は最初のメッシュヒットにのみ合成される(両インテグレータ共通の制限)。
- **supersample**: GI ポストパスは supersample 後の hi-res グリッドで走る。pt1 の
  「半解像度」はこのレンダーグリッドの半分。ベンチ・比較は `--supersample 1` で行う。
- **デノイズ**: pt1 は合成前の E バッファのみを gather 解像度で OIDN デノイズする
  (`--denoise`、albedo 乗算前 = demodulation 相当)。最終色の `--denoiser` は
  pt1 時は既定 none(明示指定は尊重)。二重デノイズ防止。

## Multi-bounce 拡張の方針(将来)

1 バウンス固定は `pt1GatherPoint` の「ヒット時 `L = oneBounceRadiance(...)`」の 1 段に
閉じている。multi-bounce 化はこの評価を再帰(またはループでレイを続けて throughput
`ρ` を掛け合わせる標準のパストレース形)に延長すればよい。cache 側の
`prevCache` 渡し(前バウンスの補間値を加算)を pt1 の full-res E バッファで模倣する
ハイブリッドも可能。
