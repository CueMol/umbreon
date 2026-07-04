# umbreon_cli レンダリング品質ガイド

`umbreon_cli` で Ambient Occlusion・ソフト影を使い、きれいな静止画を出すための設定指針。
CLI の全オプションは `umbreon_cli --help`、ライブラリ API は
[api/libumbreon.md](api/libumbreon.md) を参照。

## 推奨「高品質」設定

```sh
umbreon_cli <scene>.pov -W 700 -H 700 \
  --ao-samples 64 --ao-distance 15 \
  --shadows on --light-radius 4 --shadow-samples 24
```

## なぜきれいに効くか（最重要）

`.pov` パスは `--supersample` が既定 3（2100² で描画 → 700² へ box 平均）。AO/影はモンテ
カルロなのでノイズが出るが、box 平均によって `supersample² × samples` 分が実効的に平均化
される。上の例は AO 実効 9×64 = 576 サンプル/出力px、影 9×24 = 216。だから中サンプルでも
クリーンになる。**きれいさに一番効くレバーは supersample**。

## 各ノブの指針

| フラグ | 推奨 | 効果 / 注意 |
|---|---|---|
| `--supersample` | 3（.pov 既定）/ 大光源や最高品質は 4 | 最重要の denoise＋AA。コストは二乗 |
| `--ao-samples` | 32（十分）〜64（とても綺麗） | hemisphere レイ数。supersample と乗算される |
| `--ao-distance` | シーン依存（下記・奥行き節） | AO の届く半径（world 単位）。未指定だと auto = 0.7×bbox 対角＝大局的 |
| `--ao-intensity` | 1.0（full）/ 控えめは 0.6–0.8 | ambient 減光の強さ。> 1 は過暗化 |
| `--ao-diffuse` | 0（既定）/ 奥行き重視は 1.0 | 凹部の**直接 diffuse も減光**。陰影強度の本命レバー（下記・奥行き節） |
| `--ao-falloff` | 0（binary・推奨）/ >0 で距離減衰 | 0 が最も谷を鋭く落とす。>0 は遠方遮蔽を緩め平坦化方向 |
| `--ao-multiscale` | on（奥行き重視） | 3 段ネスト半径で接触影＋形状陰影を両立 |
| `--ao-bent-normal` | on（方向性 ambient） | bent normal 沿いの sky/ground 勾配。形状感を補強 |
| `--shadows` | on | 影を有効化 |
| `--light-radius` | 2–6 度（自然なソフト影） | エリアライトの角半径（度・シーン非依存）。0＝ハード影。大きいほど penumbra が広くノイジー |
| `--shadow-samples` | 16–32 | ソフト影のサンプル数（>1 でソフト化）。light-radius を上げたら増やす |

## 適応的アンチエイリアシング（--aa adaptive）

`--aa adaptive` は、フルグリッド supersampling の代わりに「出力ピクセル中心を 1 サンプル
だけシェード → 不連続（geomID/セクション境界・色コントラスト・法線差・深度段差）を検出
→ 境界ピクセルだけ全サブピクセル評価、平坦領域は複製」を行う決定論的な適応 AA。
ジッターなし・スレッド数不変で、**フラグされたピクセルはグリッド描画とビット一致**、
複製ピクセルは補間丸め（~1 ulp）以内。既定は `--aa grid`（従来のフルグリッド、
フラグ無し出力はバイト一致のまま）。

```sh
# 高速化: ss=3 の品質のまま境界だけ 3x3
umbreon_cli <scene>.pov --supersample 3 --aa adaptive

# 品質モード: ss=1 のコスト帯で境界だけ 3x3 相当の AA
umbreon_cli <scene>.pov --supersample 1 --aa adaptive --aa-depth 3

# 閾値チューニング: リファインメントマスクを PNG で確認
umbreon_cli <scene>.pov --supersample 3 --aa adaptive --aa-debug on
```

| フラグ | 既定 | 効果 |
|---|---|---|
| `--aa grid\|adaptive` | grid | adaptive で適応リファインメント有効化 |
| `--aa-threshold` | 0.1 | 隣接出力ピクセルの per-channel 線形色コントラスト閾値（POV の Antialias_Threshold 相当） |
| `--aa-depth` | 0 (=ss) | フラグ画素のサンプル格子。ss より大きくすると境界だけグリッドより高密度（ss の倍数に切り上げ） |
| `--aa-debug` | off | マスク AOV を `<出力名>_aaMask.png`（`--dump-aov` 指定時はその prefix）に出力 |

コスト特性（実測 700²、8 スレッド）:

- 効果は**フラグ率**（境界ピクセルの割合）と**ピクセルあたりのシェーディングコスト**で
  決まる。シェーディングが重いほど、平坦部を間引く効果が大きい:
  - 透明シーン（front-to-back walk）: 約 3.6〜4.4 倍
  - 影付きサーフェス: 約 2.4〜3.6 倍
  - AO 支配（`--ao-samples 32` 等）: ほぼ中立〜わずかに劣る（下記）
- **AO/ソフト影はコスト中立**（設計上の決定）: 複製ピクセルは中心 1 点を
  `samples × ss²` 本で評価する（フラット領域の実効サンプル数＝グリッドの box 平均と
  同等、ノイズ品質を維持）。AO レイ総数がグリッドと同等なので高速化は無く、
  マスク用の中心評価のオーバーヘッド分だけ数%遅くなることがある。AO 自体の削減は
  別施策（AO 評価グリッドの分離）が必要。
- `--supersample 1 --aa adaptive --aa-depth 3` は ss=3 グリッド比で約 4 倍速く、
  平坦部は ss=1 と同一・境界のみ 3×3 平均（PSNR ~47dB vs ss3、素の ss1 は ~43dB）。
- `--edges` と併用可: shading 済みサブピクセル（フラグセル・未フラグ中心）は
  integratePixel の first-hit から、残りの複製サブピクセルはシェーディング無しプローブ
  から G-buffer を得るため、各サブピクセルは 1 回だけトレースされ、抽出される
  ストロークはグリッド描画と**ビット一致**。edges のコストは概ね中立（±10%）。
  ストロークの AA は従来どおり supersample 由来なので、`--edges` では
  `--supersample 3` を維持すること（`--aa-depth` はストロークには効かない）。
- `--gi` とは未対応（警告を出してグリッドにフォールバック）。

## 粗解像度 AO（--ao-res out）

`--ao-res out` は、AO ギャザーを**出力ピクセルごとに 1 回**（supersample 前の解像度、
pt1 の `--indirect-res out` と同じ発想）だけ実行し、各ヒットが法線・深度ガイド付きの
bilateral 補間で参照する。ガイドが合わない画素（シルエットのリム・透明の奥層・
セル未満の細部）は**その場で従来どおりの正確な inline ギャザーにフォールバック**し、
supersample の box 平均がそのままデノイズする。AO レイ数は平滑領域で約 1/ss²
（ss=3 で 1/9）。既定は `full`（従来の per-hit ギャザー、バイト一致）。

```sh
# AO を含む標準構成の高速化
umbreon_cli <scene>.pov --supersample 3 --ao-samples 32 --ao-res out --ao-ld on

# フォールバック率の確認（<出力名>_aoPatchMask.png を出力）
umbreon_cli <scene>.pov ... --ao-res out --ao-res-debug on
```

| フラグ | 既定 | 効果 |
|---|---|---|
| `--ao-res full\|out` | full | out で粗解像度ギャザー + per-hit 補間 |
| `--ao-res-debug` | off | フォールバック（inline パッチ）マスク AOV を PNG 出力 |

注意点:

- **実効サンプル数のトレードオフ**: full は出力 1px あたり `aoSamples × ss²` 本の
  box 平均、out は `aoSamples` 本の補間。GTAO レシピ級（`--ao-samples 256`）では
  実質差なし。低サンプル（≤32）ではセル単位の斑が出うるため **`--ao-ld` 併用を推奨**
  （セル内分散削減 + per-pixel Cranley-Patterson 回転でセル間非相関。ただし
  `--ao-ld` は enhanced 推定器に切り替わるため、legacy 出力とはビット非互換）。
- ss=1 では out は inline に退化（粗グリッド = レンダ解像度なので正しい意味論）。
- `--gi` とは未対応（警告を出して full にフォールバック）。
- 適応 AA（`--aa adaptive`）と併用可: coarse AO 有効時は複製中心の AO ブースト
  （samples×ss²）が不要になり自動で無効化されるため、**AO シーンでも適応 AA の
  高速化が効くようになる**（従来は AO コスト中立）。
- メモリ: 粗グリッドは約 48B ×（W/ss）×（H/ss）。

### GTAO レシピ vs pt1（速度の目安）

深み表現が目的なら **pt1 の方が速く、拡大時のざらつきも無い**（AO 経路にはデノイザが
無く supersample 平均だけが頼りなので高サンプルが要るのに対し、pt1 は out-res gather +
LD + OIDN デノイズで 8〜32spp で済む）。ao_test1 700² ss=3 の実測 wall time:

| 手法 | wall | 備考 |
|---|---|---|
| GTAO レシピ full（256spp） | 53.6s | 従来 |
| GTAO レシピ `--ao-res out` | 7.7s | 見た目ほぼ同等、拡大でざらつき |
| GTAO `--ao-res out` + 64spp + `--ao-ld` | 2.3s | GTAO の見た目を保つ現実解 |
| `--integrator pt1 --quality draft` | 1.5s | GI（トーンは別物）、デノイズ済み |
| `--integrator pt1 --quality high` | 3.6s | 同上、2 バウンス |

## 奥行きを強く出す（OpenGL GTAO 相当）

分子表面で「凹は暗く・凸は明るく」をはっきり出すための実測レシピ。既定の AO
（`--ao-diffuse 0`・auto 大半径）は AO OFF と全体の数%しか変わらず**陰影が弱い**ので、
奥行き目的では以下を明示する。

```sh
umbreon_cli <scene>.pov -W 700 -H 700 --supersample 3 \
  --ao-samples 256 --ao-multiscale on --ao-bent-normal on \
  --ao-falloff 0 --ao-diffuse 1.0 --ao-distance 40
```

効くレバーの順（OpenGL GTAO 参照と並べた検証より）:

1. **`--ao-diffuse 1.0`** — 本命。AO は既定で ambient 項しか減光しないが、CueMol 既定
   ライティングはエネルギーの大半が direct light にあるため、direct diffuse も凹部で
   減光して初めて深い陰影が出る。
2. **`--ao-falloff 0`（binary）** — 谷を最も鋭く落とす。`>0` の距離減衰は遠方遮蔽を緩め、
   コントラストを下げて平坦化する。
3. **`--ao-distance` は大きめ** — このスケールの分子シーンでは**大半径ほど強い**
   （シーン径の 0.5〜0.85 倍）。短半径は近傍に occluder が無く効かない（直感に反する）。
   起動時ログの auto 値（`ambient occlusion: ... radius XX`）を基準に上下させて調整。
4. 補助: `--ao-intensity` は 1.0 を基準に（>1 は過暗化）。淡色凹部の黒潰れが気になるときは
   `--ao-multibounce on`。

> **注意（ambient 配分は AO の効きを上げない）**: AO が弱いからと言って ambient のエネルギー
> 配分（POV の `_amb_frac` 等）を上げても、その分は GI 専用に回り direct が減るだけで、AO
> モードでは全体が暗くなりコントラストはむしろ下がる。強度は `--ao-diffuse` で上げること。

> **CPK モデルも暗化する**: AO は SES メッシュだけでなく**実プリミティブ（VdW 原子球・bond
> シリンダ）にも適用される**ので、ポケット内のリガンドも周囲の表面と同様に暗くなる。NPR の
> outline 装飾（シルエットエッジ）はフラットのまま。

## バリエーション

最高品質（広めのソフト影も滑らか）:

```sh
umbreon_cli <scene>.pov -W 700 -H 700 \
  --supersample 4 --ao-samples 96 --ao-distance 15 \
  --shadows on --light-radius 6 --shadow-samples 48
```

高速プレビュー:

```sh
umbreon_cli <scene>.pov -W 700 -H 700 \
  --supersample 2 --ao-samples 16 --ao-distance 15 \
  --shadows on --light-radius 3 --shadow-samples 8
```

## 見積もりと決定論

- 乱数は `(pixel, sample)` シードで決定論的（再描画しても同一・TBB スレッド数に依存しない）。
- 総二次レイ数 ≈ `supersample² ×（ao-samples + ライト数 × shadow-samples）`。
  品質とレンダ時間はこの式で見積もれる。
- 並列度は `--threads`（`0` = 全コア, `1` = シリアル）。再ビルドなしで速度比較できる。

## diffuse GI インテグレータ（--gi / --integrator）

diffuse GI は 2 つのインテグレータを選べる(設計と規約は
[pt1_design.md](pt1_design.md)、比較は `scripts/compare_integrators.sh`)。

```sh
# irradiance cache(既定)
umbreon_cli <scene>.pov --gi on

# pt1: パストレース per-pixel gather(実験的、cache の ground-truth 参照)
umbreon_cli <scene>.pov --integrator pt1 --spp 8
```

| フラグ | 既定 | 効果 / 注意 |
|---|---|---|
| `--integrator <cache\|pt1>` | cache | 間接光インテグレータの選択。`pt1` は `--gi on` を含意 |
| `--spp <int>` | 8 | pt1: ピクセルあたりの gather レイ数 |
| `--indirect-res <full\|half\|quarter\|out>` | half | pt1: gather 解像度。レンダーグリッド（supersample 後）の 1/{1,2,4}、`out` は最終出力サイズ。full 以外は joint bilateral upsample + シルエットリムの full-res パッチ（`--pt1-edge-patch`） |
| `--pt1-edge-patch <on\|off>` | on | 縮小 gather グリッドが解決できないシルエット縁ピクセルを full-res で再 gather |
| `--pt1-patch-thresh <w>` | 0.3 | パッチ対象の upsample 重み閾値（大きいほど広く・高品質・低速） |
| `--pt1-stats <on\|off>` | off | OIDN 段階分解（device/filter/execute）を stderr に出力 |
| `--denoise <on\|off>` | on | pt1: 間接照度バッファのみを OIDN デノイズ(direct/albedo は触らない) |
| `--sky <uniform\|gradient>` | uniform | pt1: gather の sky モデル。gradient は天頂=`--sky-radiance`、地面=`--ao-ground` |
| `--sky-radiance r,g,b` | 1,1,1 | pt1: sky のティント(ambient エネルギーに乗算) |
| `--seed <int>` | 0 | pt1: 決定論的 per-pixel RNG シード |
| `--gi-max-dist <world>` | 0 | gather レイの最大距離。cache: auto=0.1×対角 / pt1: auto=∞(意図的な差) |
| `--pt1-upsample-normal-pow <f>` | 32 | upsample の法線 edge-stop 指数 |
| `--pt1-upsample-depth-scale <f>` | 0.02 | upsample の深度 edge-stop スケール |

注意:
- pt1 と `--env-light` の併用は sky の二重計上(警告が出る)。GI の sky は
  `--sky` / `--sky-radiance` / `--gi-env-intensity` で制御する。
- ベンチ・A/B 比較は `--supersample 1` で行う(GI は supersample 後のグリッドで走るため)。
- pt1 実行時は段階別時間が stdout と `outputs/timing.json` に出る。
