# 指示: umbreon への Tone Hatching NPR モード追加 — 実装プランの作成

**これはプラン本体ではなく、プランを書くための指示書です。**
あなたのタスクは umbreon のコードベースを詳細に調査し、`docs/plans/npr-tone-hatching.md` として
実装プランを作成することです。**この時点でコードは一切変更しないでください。**

`docs/plans/` の既存文書（特に `edge-extraction-screenspace.md`）が
このリポジトリのプラン文書の水準です。同じ水準を満たしてください。すなわち:

- すべての主張に `file:line` を付け、作業ツリーに対して実際に検証する
- ブリーフ（この文書）の記述が現状と食い違っていたら、**食い違いを明示的に指摘する**。
  この指示書は外部での設計議論に基づいており、細部が現状とずれている可能性がある
- 既定パス（機能 OFF）が **byte-identical** であることを設計の中心に置く

---

## 1. 何を作るのか

`--hatch` で有効になる NPR シェーディングモードを追加する。
陰影を連続階調ではなく、**線の密度**（ハッチング／クロスハッチ／網点）で表現する。
Praun et al. *Real-Time Hatching* (SIGGRAPH 2001) の Tonal Art Map の考え方を、
テクスチャを持たない手続き的生成として実装する。

想定用途は CueMol の論文図・プレゼン図（静止画）。リアルタイムである必要はなく、
CPU オフラインである利点（解析的評価、任意の反復回数）を積極的に使ってよい。

既存の `--edges`（stroke edges）と併用できること。ハッチが階調を、
エッジが輪郭を担当する構成が最終的な絵になる。

---

## 2. 合意済みの設計判断（前提として扱う）

以下は外部の設計議論で合意済み。**プランはこれらを出発点とすること。**
ただし調査の結果これらが技術的に成立しないと判明した場合は、
黙って別案にせず「なぜ成立しないか」を明記したうえで代案を提示すること。

### 2.1 パイプライン上の位置 — ここが最重要

トーンの**生成**とインクの**消費**を分離する。

- **生成**: `hit_shader` の中。AO 計算と同じ場所（`aoFactor` / `diffuseAo` が求まる直後）。
  primary ray ループに融合されている既存構造に乗せる。別パスにしない。
- **消費**: `renderFrame()` の最終段。`applyAssumedGamma` の**後**。

```
EmbreeRenderer::render()
    Setup → CoarseAo → Primary ──┬── color
                                 ├── contactAo / shapeAo
                                 └── hatchTone / hatchMask   ← 生成
    GlobalIllum                                (NPR では skip)
  ↓ fog
  ↓ applyStrokeEdges (hi-res)
  ↓ boxDownsample(ss)   ← hatchTone / hatchMask も最終解像度へ
  ↓ denoise                                    (NPR では skip)
  ↓ applyAssumedGamma
  ↓ applyHatch(frame)   ← 消費
```

**ストロークエッジとは意図的に逆であることに注意。** エッジは
「アンチエイリアスのため box-downsample の前に hi-res で合成」しているが、
ハッチを hi-res で打ってから box 平均すると二値のインクが灰色に潰れ、
かつモアレが出る。正しい順序は
「トーンを hi-res で作る → downsample（＝トーン信号の正しい AA）→ 最終解像度で二値化」。
これにより ss を上げるほどトーンが滑らかになり、線幅は出力ピクセル基準で一定に保たれる。

`applyAssumedGamma` の後に置くのは、インク色を表示色として指定したいため。
純二値インクはガンマ不変（0^g=0, 1^g=1）なので位置の自由度はあるが、
色インクを扱うので表示空間で合成するのが正しい。
`image_ops.hpp` の group-alpha マルチパス合成が
「CueMol の blendpng と同じく表示エンコード済みの値を合成しなければならない」
としているのと同じ判断。

### 2.2 新規 AOV

`FrameResult` に追加:

| AOV | 型 | 内容 |
|---|---|---|
| `hatchTone` | `vector<float>` w·h | 陰影トーン（連続値、リニア） |
| `hatchMask` | `vector<float>` w·h | ヒット被覆率。1=サーフェス、0=背景 |

既存 AOV と同じ規約に従う: **`opt.hatch.enable` のときだけ sized/written、
それ以外は空**。`pipeline.cpp` の downsample リストに両方を追加（channels=1）。

`hatchMask` は必須。`depth` / `viewZ` / `objectId` は downsample されないため
最終解像度では使えない。ヒット 1.0 / 背景 0.0 を box 平均することで、
シルエット境界でのハッチのアンチエイリアスがそのまま得られる。

### 2.3 手続き的 TAM — 実装の核

素朴な多層 multiply（記事や既存 Unity 実装の多く）はトーン変化で線が入れ替わり
ポップする。テクスチャを持たずに TAM の入れ子性を得るには、
**線のインデックスから出現レベルを決める**。

基本間隔 `S`、細分レベル数 `K`、線を `j * S / 2^K` の格子上に置くとき:

```cpp
static inline int firstLevel(int j, int K) {
  if (j == 0) return 0;
  unsigned u = static_cast<unsigned>(j < 0 ? -j : j);
  int ntz = 0;
  while (!(u & 1u)) { u >>= 1; ++ntz; }
  return ntz >= K ? 0 : K - ntz;
}
```

「線 `j` を描くのは `tone < T[L(j)]` のときだけ」（`T` は `L` について単調減少）とすれば、
トーンが暗くなるにつれ既存の線の隙間に新しい線が挿入され、
**一度現れた線は決して消えない**。TAM のトーン方向・スケール方向の入れ子性が
ビット演算ひとつで得られる。

1 レイヤの被覆率（`x, y` は FINAL 解像度のピクセル座標）:

```cpp
float layerInk(const HatchLayer& L, float x, float y, float tone) {
  const float a = L.angleDeg * float(M_PI) / 180.0f;
  const float u = -std::sin(a) * x + std::cos(a) * y;   // 線に垂直な軸
  const float step = L.spacingPx / float(1 << L.subdiv);
  const float half = 0.5f * L.widthPx;
  const int j0 = int(std::floor((u - half) / step));
  const int j1 = int(std::floor((u + half) / step));
  float cov = 0.0f;
  for (int j = j0; j <= j1; ++j) {
    const int lv = firstLevel(j, L.subdiv);
    const float t = L.toneHi + (L.toneLo - L.toneHi) * (float(lv) / float(L.subdiv));
    if (tone >= t) continue;                                    // 未出現
    const float fade = std::min(1.0f, (t - tone) * L.fadeInv);  // 幅で滑らかに導入
    const float w = half * fade;
    const float d = std::fabs(u - float(j) * step);
    cov = std::max(cov, 1.0f - smoothstep(w - 0.5f, w + 0.5f, d));
  }
  return cov;
}
```

内側ループは常に 3〜5 反復程度（`half / step` に比例）。
`fade` は Webb et al. *Fine Tone Control in Hardware Hatching* (NPAR 2002) が
やっていることの手続き版で、線を幅 0 から生やすことで階調の飛びを消す。

**網点（スクリーントーン）も同じ枠組みで扱う。** 網点は「方向を持たない
2 次元格子上のドット」なので、`layerInk` を 2 次元セルの距離場に置き換えれば
同じ nesting の議論がそのまま使える（ドットの出現レベルを 2 次元細分で割り当てる）。
線と点を同一の `HatchLayer` に `LayerKind { Line, Dot }` として統合すること。
詳細は §2.4。

### 2.4 マークのスタイル — 格子とマーク形状の分離

§2.3 の nesting の議論は「マークが**どこに**現れるか」だけを決めている。
**どんな形のマークか**はそれと完全に直交する。この分離を実装の構造に反映すること。

3 層に分ける。

1. **格子 (lattice)** — マークの配置と `firstLevel` によるレベル割り当て。
   線なら 1 次元（垂直軸上の位置）、点なら 2 次元（2 次元細分）
2. **マーク形状 (profile)** — 格子要素までの距離と成長パラメータから被覆率を返す
3. **摂動 (perturbation)** — 手描き感。位置ジッタ、線幅変調、途切れ、揺らぎ

**絶対条件: 摂動は tone に依存してはならない。** 格子インデックス `j` と
沿線座標のみの決定的関数にすること。tone に依存させると、tone が変わったときに
マークが動いて nesting が壊れ、線がポップする。§2.3 の保証は
「一度現れた線は消えない」だけでなく「動かない」も含む。

#### 2.4.1 点のスタイル

円・正方形・菱形は Lp ノルムの p を変えるだけで連続的に得られる。
形状を離散的な enum の羅列にせず、1 本の連続パラメータに畳むこと。

```cpp
// shapeExponent: 1=菱形, 2=円, 4=角丸正方形, >=16 で実質正方形
float dotDistance(float dx, float dy, float p) {
  if (p >= 16.0f) return std::max(std::fabs(dx), std::fabs(dy));   // L∞
  return std::pow(std::pow(std::fabs(dx), p) + std::pow(std::fabs(dy), p), 1.0f / p);
}
```

**単位面積定数を形状ごとに補正すること。** 半径 r の 2 次元 Lp 球の面積は
`A_p · r²`, `A_p = (2·Γ(1+1/p))² / Γ(1+2/p)`（p=1: 2, p=2: π, p=∞: 4）。
tone → 半径のマッピングを `r = sqrt((1 - tone) / A_p)` としないと、
**形状を変えただけで見かけの濃度が変わる。** 定数は起動時テーブルで持てばよい。

**被覆率 50% を超えたら反転すること。** 円ドットは半径が spacing/2 に達した時点で
被覆率 π/4 ≈ 78.5% で頭打ちになり、それ以上暗くならない。実際の AM スクリーンと同じく、
50% を超えたら「黒地に白い穴を成長させる」側へ切り替える。これをやらないと
最暗部が出せず、分子図では影の底が抜ける。

**ジッタでスティップルになる。** 格子は保ったまま、各ドットをハッシュ由来の
オフセットで動かす。ドットの識別子とレベルは変わらないので nesting は厳密に保たれる。
`jitter = 0` が規則的なハーフトーン（スクリーントーン）、`jitter ≈ 0.4` で
科学イラスト的なスティップル。**パラメータ 1 本で両端をカバーできる**ので、
Deussen / Secord 系の重み付き Voronoi スティップリングを別実装する必要はない。

#### 2.4.2 線のスタイル

- **断面プロファイル** — `smoothstep` のエッジ幅。0 に近ければ製図ペン、
  広げれば鉛筆
- **揺らぎ (wobble)** — 沿線座標 v の滑らかな 1D ノイズで u を摂動:
  `u += amp * noise(v / wavelength + hash(j))`。**実装コストがほぼゼロで
  手描き感への寄与が最大。** `hash(j)` を入れて線ごとに位相をずらすこと
- **線幅変調** — `w *= 1 + a * noise(v / L + hash(j))`。筆圧感
- **途切れ / ストローク長** — 沿線方向の duty 関数でマスクする。
  鉛筆ハッチが「有限長のストロークの集まり」に見えるのはこれによる。
  ストロークの端はテーパーを掛ける
- **紙目 (tooth)** — 被覆率全体に細かいノイズを乗算。鉛筆・木炭の質感

すべて `hash(j)` と沿線座標の関数であること。tone は入れない。

#### 2.4.3 型設計

```cpp
enum class LayerKind { Line, Dot };

struct MarkStyle {
  // --- 共通 ---
  float edgeSoftness  = 0.5f;   // smoothstep 幅 (px)。0=硬い, >1=鉛筆
  float toothAmp      = 0.0f;   // 紙目ノイズ振幅
  float toothScalePx  = 3.0f;   // 紙目の空間周波数
  unsigned seed       = 0;      // 決定性を保つ明示シード
  // --- Dot ---
  float shapeExponent = 2.0f;   // 1=菱形, 2=円, 4=角丸, >=16=正方形
  float dotAspect     = 1.0f;   // 楕円化
  float dotAngleDeg   = 0.0f;   // 非円形マークの回転
  float jitter        = 0.0f;   // 0=ハーフトーン, ~0.4=スティップル
  bool  invertAbove50 = true;   // 50% 超で白穴成長へ切替
  // --- Line ---
  float wobbleAmpPx   = 0.0f;
  float wobbleWavePx  = 40.0f;
  float widthJitter   = 0.0f;   // 相対
  float strokeLenPx   = 0.0f;   // 0=無限長, >0 で途切れ
  float strokeGapPx   = 0.0f;
  float strokeTaper   = 0.3f;   // 端のテーパー比
};
```

`HatchLayer` に `LayerKind kind` と `MarkStyle mark` を持たせる。
`layerInk` は kind で分岐するが、**nesting のロジックと tone→成長量のマッピングは
共有すること**（`firstLevel` と閾値テーブル `T[]` は kind に依存しない）。
これが守れているかがこの設計が正しく分離できている指標になる。

#### 2.4.4 プリセットが一次インタフェース

パラメータは 15 本を超える。CueMol の UI から 15 本のスライダを触らせるのは
現実的でないので、**名前付きプリセットを一次インタフェースにする。**
個別パラメータはプリセットに対する上書きとして位置づける。

| 名前 | 構成 |
|---|---|
| `pen-cross` | 3 レイヤ / Line / 45°, -45°, 0° / 硬いエッジ / 摂動なし |
| `pencil` | 2 レイヤ / Line / 揺らぎ + 線幅変調 + 途切れ / 紙目あり |
| `engraving` | 1 レイヤ / Line / 線幅変調のみ / subdiv 大 |
| `stipple` | 1 レイヤ / Dot / jitter 0.4 / 円 |
| `screentone-60` | 1 レイヤ / Dot / jitter 0 / 円 / 45° / spacing = 60 線相当 |
| `manga-square` | 1 レイヤ / Dot / shapeExponent 16 / 45° |

CLI は二段構え:

```
--hatch-style ID=pencil                       # セクションにプリセットを割り当て
--hatch-layer ID:0 wobble=1.5,strokeLen=18    # 個別上書き
```

`--edge ID=spec` のパーサ（`=` 分割、`_show` 前置除去、`geo.groupNames` 解決、
`--list-groups` 発見 UX）をそのまま流用できるはず。

**プリセット表の実際の数値をプランで確定させること。** これは実装の細部ではなく
プロダクト仕様なので、上の表を具体的な `HatchLayer` / `MarkStyle` の値まで
埋めた形でプランに載せること。既定プリセットが `pen-cross` でよいかも判断すること。

### 2.5 トーンの作り方

トーンは `frame.color` の輝度**ではなく**、照明成分のみから作る。
CueMol の図は鎖や二次構造で色分けされているため、色輝度をトーンにすると
濃い青のヘリックスは真っ黒に、黄色のヘリックスはほぼ白になり、
照明が同じでも色が違うだけで陰影が変わってしまう。

```cpp
struct ToneRecipe {
  float diffuseWeight  = 1.0f;   // 全ライトの saturate(N·L) 合計（影の減衰込み）
  float ambient        = 0.12f;  // 下駄。0 だと影が真っ黒に潰れる
  float contactAoPow   = 1.0f;   // 接触 AO の効かせ方
  float shapeAoPow     = 0.6f;   // 形状 AO の効かせ方
  float blackPoint     = 0.0f;
  float whitePoint     = 1.0f;
  float gamma          = 1.0f;   // トーンカーブ
  float specularCut    = 0.0f;   // >0: 鏡面がこの値を超えた画素を紙白に抜く（既定 0=無効）
};
```

`contactAo` と `shapeAo` を独立に持たせるのが分子図では効く。
接触 AO を強めると原子の隙間や溝が線で埋まり、形状 AO を強めるとドメイン全体の
凹凸が読める。両者の最適点は求める図によって違うので 1 本のスライダにまとめない。

`applyHatch` の入口で `srgbEncodeF(tone)` を通してから閾値に掛ける
（`image_ops.hpp` に既存、追加実装不要）。リニア輝度のまま `T[]` と比較すると
中間調が極端に暗く出る。トーン量子化（`toneLevels`）はその後。

**hit_shader のどこで何を集約すればこのレシピが計算できるかを、
実コードを読んで具体的に特定すること。** `aoWriteAov` が
`hs.contactAo` / `hs.shapeAo` を書いている箇所が起点になるはず。
拡散項が per-light でどう積算されているか（`shading.hpp` の POV local illumination）を
確認し、トーン用に取り出せる形になっているかを判断する。

### 2.6 ベタ塗り / 紙 と インク色 — 直交する 2 択

「ベタ塗りの上に黒でハッチ」と「紙白に指定色でハッチ」は、2 つのモードではなく
**紙側とインク側の独立した選択**として持たせる。組み合わせ 4 通りすべてが実用的。

| 紙 (base) | インク | 見た目 |
|---|---|---|
| 紙白 | 黒固定 | ペン画・モノクロ論文図 |
| 紙白 | オブジェクト色 | 色鉛筆／カラートーン風 |
| ベタ塗り (albedo) | 黒固定 | 劇画・アメコミ風 |
| ベタ塗り | albedo を暗くした色 | 同系色の陰影、印刷寄り |

```cpp
enum class HatchBase { Paper, Albedo };
enum class HatchInk  { Fixed, FromAlbedo };
```

**合成は乗算 1 種類だけ実装すればよい。** 被覆率 `c`、インク色 `I`、下地 `B` として

```
multiply:  out = B * (1 - c * (1 - I))
over:      out = B * (1 - c) + I * c
```

`I = 0`（黒）では両者一致。色インクでのみ差が出て、そこでは乗算が正しい
（同色のクロスハッチが交差した箇所が `I²` になって濃くなる＝実際の色鉛筆・
カラートーンの重ねと同じ）。

```cpp
float f[3] = {1, 1, 1};
for (const auto& L : layers) {
  const float c = layerInk(L, x, y, tone) * L.opacity;
  for (int k = 0; k < 3; ++k) f[k] *= 1.0f - c * (1.0f - I[k]);
}
for (int k = 0; k < 3; ++k) out[k] = B[k] * f[k];
```

**ベタ塗りの下地は `frame.color` ではなく `frame.albedo`。**
`frame.color` は既に `N·L` と AO で陰影が付いているので、その上にハッチで
もう一度陰影を乗せると階調が二重計上され暗部が潰れる。ハッチングの前提は
「下地はフラット、階調は線の密度だけが担う」こと。
`frame.albedo` は `hit_shader` が第一ヒットの `mat.diffuse * pigment` を
捕まえている AOV で、既に downsample 対象。
`base = Albedo` のとき軽い量子化（`round(a*N)/N`, N=4 程度）を掛けられるようにすると
より「ベタ」らしくなる。

### 2.7 コントラスト保証 — 実務上の要

4 通りのうち 2 通りは素のままだと絵が消える。紙白に淡い黄色のリガンドを
色インクで描いても見えないし、濃紺のヘリックスをベタ塗りにして黒でハッチしても見えない。
どちらも「下地とインクの輝度差がないとトーンが伝わらない」という同じ問題。
ハッチは密度で階調を作るので、線が見えなければ階調がゼロになる。

合成前に、下地に対してインク側の輝度を必要なだけ動かす:

```cpp
float lb = luma(B), li = luma(I);          // 表示空間の輝度で比較
if (lb - li < opt.inkMinContrast) {
  const float target = std::max(0.0f, lb - opt.inkMinContrast);
  const float s = (li > 1e-4f) ? target / li : 0.0f;
  for (int k = 0; k < 3; ++k) I[k] *= s;   // 色相・彩度を保ったまま暗くする
}
```

インク側だけを動かすこと。下地を動かすと分子の色分けが崩れる。
`inkMinContrast` は 0.25 前後が実用的。

下地が既に暗く（`lb < inkMinContrast`）この式で逆方向に動かせない場合は、
**インクを `lb + inkMinContrast` の明度に持ち上げる**（暗い面に明るいハッチ＝版画的）
と決め打ちしてよい。オプションにしない。

### 2.8 セクション別スタイル

分子図ではこれが本領。既存の `Scene::groupEdgeStyle` とまったく同じ形で
`Scene::groupHatchStyle` を持たせる。

```cpp
struct GroupHatchStyle {
  bool enable = true;
  HatchBase base;  HatchInk ink;
  float inkColor[3];
  int   layerMask = 0x7;    // 使うレイヤの選択
  float toneScale = 1.0f;   // このセクションだけトーンを持ち上げる／落とす
};
```

「リボンはベタ塗り＋黒のクロスハッチ、リガンドは紙白＋分子色の細いハッチで浮かせる、
表面はごく薄い一方向ハッチだけ」といった塗り分けが CueMol のセクション設定から
自然に降りてくる。**グローバルオプションではなくセクション単位である必要がある。**

CLI からの指定は `--edge ID=spec` / `--alpha ID=value` のデータパスをそのまま踏襲する
（`geo.groupNames` 解決、`--list-groups` による発見 UX）。

### 2.9 GI / デノイザの扱い

**GI は不要。** `renderFrame` 冒頭で正規化する。既存の
`aaMode == 1 && gi` / `aoResDiv > 1 && gi` のフォールバックと同じ形に揃えること。

```cpp
if (hi.hatch.enable && hi.hatch.mode == HatchMode::Ink) {
  if (hi.gi) {
    logMessage(LogLevel::Warning, "--hatch ink does not use GI; disabling --gi");
    hi.gi = false;
  }
  hi.denoiser = DenoiserBackend::None;
  hi.pt1Denoise = false;
}
```

**置き場所は `progress->setPhasePlan(...)` の前。** 既存の 2 つの正規化がそこにあるのは
コストモデルが正規化後のオプションを見て進捗バーの重み付けを決めるため。
GI を切ったのにフェーズプランが GI に 75〜90% を割り当てたままだとバーが動かなくなる。

**ただし `HatchMode { Ink, Over }` の区別が要る。** GI を無条件で切ってよいのは、
`frame.color` を完全に捨てる純インク画（`--edges-only` が背景色で塗り潰しているのと
同じ発想）の場合だけ。フラットな色面の上にハッチを重ねる `Over` では色が最終画に残る
（`data/1ab0_scene4_toon1.pov` のような toon 系がそれ）。
既定は両モードとも GI オフ、`Over` では明示指定を尊重、程度が妥当。

**色デノイザは NPR でも通さないままでよい。** 理由を 2 つプランに明記すること:

1. `denoiseAtrous` は `denoiseDemodulateAlbedo` で color/albedo を分離してから
   照度側を平滑化する構成で、「照度が低周波、albedo が高周波」を前提にしている。
   ハッチのトーンは `hit_shader` で独立に書く照度そのものなので、
   色に掛けたデノイザの効果はトーン AOV に届かない
2. OIDN は学習ベースで、低周波照度向けにチューニングされており
   `denoiseSigmaL` のような明示的制御もない。二値化直前のトーンには挙動が読めない

### 2.10 二値化とノイズ — AO 側で解く

GTAO のノイズは半球 1 バウンスの遮蔽率という有界量なので分散が小さく、
連続階調なら ss の box 平均だけで実用上十分（だからデノイザ既定オフは正しい）。
崩れるのは二値化を挟んだときだけで、閾値 `T[L]` 近傍の画素が全振れして
ハッチの中に胡麻塩状の飛び線が出る。**閾値近傍の画素だけの問題**なので、
全画面デノイズは要らない。既存の機構で解く。

- **`aoResDiv` の既定を NPR モードでは `-1`（出力解像度）にする。**
  `ao_coarse.hpp` の構成（出力解像度セルごとに 1 回ギャザー＋法線/深度ガイド付き
  バイラテラル lookup、ガイドが棄却したヒットだけインラインギャザーへ）は
  実質的にトーン専用デノイザとして機能する。平滑域は構成的にノイズが消え、
  エッジは正確。AO レイ数も `1/div²` に落ちるので、浮いた予算を `aoSamples` に回せる。
  GI を切る以上、既存の `aoResDiv > 1 && gi` 制約とも競合しない
- **`aoLowDiscrepancy` を NPR モードで既定オン。** コストゼロで分散だけ下がる
- **フォールバック画素（シルエット縁）の追加サンプリング。**
  `pt1EdgePatchSppMul` と同じ発想。フォールバックは全体の数パーセントなので
  そこだけ `aoSamples` を 4 倍にしてもコストはほぼ無視できる。
  `aoResDebug` の `aoPatchMask` で該当画素は既に取れている。
  構造的に正しい置き場所は `HatchOptions` ではなく AO 側
  （`aoResFallbackSppMul`, 既定 4）
- **最後の手段: `HatchOptions::toneSmooth`。** `denoiseAtrous` の SVGF エッジストップを
  1 チャンネルの `hatchTone` に適用する薄いラッパ。ガイドは `normal` / `position` を共用。
  **`RenderOptions::denoiser` とは別の名前にすること** — これは画像のデノイズではなく
  トーン信号のローパスであり、同じ名前の下に置くと
  「NPR で色デノイザを切っている」意図と混ざって後から読めなくなる

優先順位はこの順。まず `aoResDiv = -1` + `aoLowDiscrepancy` + ss=3〜4 で
実際に飛び線が見えるかを確認する段階をプランに含めること。

### 2.11 fog

`applyFog` は hi-res で `frame.color` に掛かるが `hatchTone` は素通りする。
「色は霞んでいるのに線の密度は手前と同じ」という絵になる。
ただし、今のsilhouette線の実装はfogがかかると色が薄くなるようになっているので、
それと合わせるため、色は霞んでいるのに線の密度は手前と同じ、を許容することにする。

将来的に、fogがかかっている様子自体をtoneで表すことも考えるが、この場合、silhouette線にはfogをかけないなどの対処が必要（なのでこれはスコープが意図する）

### 2.12 座標系 — 今回はスクリーン空間のみ

`layerInk` の `x, y` には最終解像度のピクセル座標をそのまま入れる。
物体空間の UV パラメータ化（Praun らの lapped texture）は**今回のスコープ外**。

ただし将来 CueMol renderer 側から UV を供給する構想があるので、拡張点だけ用意する:

- `FrameResult` に `std::vector<float> hatchUv;`（w·h·2）を optional AOV として設計に含める
  （今回は実装しなくてよいが、**空ならスクリーン座標にフォールバック**する
  分岐構造にしておく）
- `layerInk` の引数を `(u_coord, v_coord)` に一般化しておけば呼び出し側の分岐だけで切り替わる
- UV の画面微分（隣接ピクセル差分）で `step` をスケールすれば
  遠景で線が詰まる問題も同じ関数内で処理できる — 設計メモとして記載

### 2.13 public API の形

`renderFrame` の最終段は薄いラッパにして、実体はレンダラ非依存の image op にする。

```cpp
// npr/hatch_shade.hpp — public に出す
void applyHatch(int w, int h, float* rgba,
                const float* tone, const float* mask,
                const float* albedo,          // base=Albedo / ink=FromAlbedo 用、null 可
                const HatchOptions& opt);
```

`applyStrokeEdges` が `FrameResult` を受け取る internal 関数なのに対し、
こちらは意図的にプレーンなポインタ引数にする。利点:

- CueMol 側で手描き／レタッチしたトーンを流し込める
- ユニットテストが合成トーン（水平グラデーション）だけで書ける。
  「トーンを連続に振るとインクが単調非減少」という入れ子性の検証は
  この関数を直接叩くのが最も確実
- 将来 UV を足すときも同じ関数に引数を 1 つ増やすだけで済む

透過背景（`opt.transparentBackground`）では背景が premultiplied（`bgr * a`, `a = 0`）で
書かれている。`hatchMask` が 0 の画素は触らず、`paperColor` は
**オブジェクト内部の下地としてのみ**使う（背景まで塗ると透過が潰れる）。

---

## 3. 調査して決めること（プランで結論を出す）

以下はこちらで判断していない。コードを読んで結論と根拠を書くこと。

1. **`hit_shader` のトーン算出点。** `ToneRecipe` を計算するのに必要な量
   （per-light の `N·L` 合計、影の減衰、`contactAo` / `shapeAo`、鏡面）が
   どこで揃うか。`HitShade` に何を足す必要があるか。
   透過サーフェスの front-to-back walk（`transparency.hpp`）でトーンをどう扱うか
   — 第一ヒットのみか、合成するか
2. **`hatchMask` の生成点。** `surfAlpha` が既にあるが `strokeEdges` ゲートで
   downsample もされない。流用できるか、独立に持つべきか
4. **`Scene::groupEdgeStyle` の実装パターン**（テーブルの型、CLI からの解決、
   `resolveStrokeStyle` の out-of-range フォールバック）を精読し、
   `groupHatchStyle` をどこまで対称に作れるか
5. **`aoResDiv` のフォールバック画素に追加サンプリングを入れる改修の侵襲度。**
   `ao_coarse.hpp` の決定性（座標ベースのシード、スレッド数非依存）を壊さずにできるか
6. **`strokeEdges` との相互作用。** エッジは hi-res で合成されてから downsample
   されるのでグレーの AA 線になる。ハッチは最終解像度で `smoothstep` により AA される。
   両者のインク色・見た目が揃うか。`edgesOnly` との組み合わせの意味。
   `objectSpaceEdges`（`strokeEdges` と排他）との関係
7. **最小フィーチャサイズのガード。** `spacingPx / 2^subdiv` が 2px を切ると
   `smoothstep(w-0.5, w+0.5, d)` の「1px フィルタ幅」仮定が破綻してエイリアスする。
   最細レベルを抑制するか、被覆率を解析的に平均するか。**ハッチは最終解像度で打つので
   ss を上げても box 平均では救えない** — ここは独立に解く必要がある
8. **ノイズ関数の選択。** value noise / gradient noise のどちらを自前実装するか。
   スレッド数非依存・座標のみの純関数であること（`ao_coarse` と同じ決定性要件）。
   既存コードに再利用できるハッシュ／サンプラがあるか確認すること
9. **摂動とレイヤ探索範囲の整合。** `wobble` / `jitter` はマークを格子から
   ずらすので、`layerInk` の `j0..j1` 探索窓を摂動の最大振幅分だけ広げないと
   マークが千切れる。窓を広げるコストと、振幅に上限を設ける案の比較
10. **`--quality` プリセット**（`docs/quality_presets.md`）に NPR 用の段を
    足すべきか。AO 設定が NPR 専用の既定を持つなら、そこに集約するのが筋かもしれない。
    §2.4.4 のマークプリセットとは別軸なので、混同しない命名にすること

---

## 4. プラン文書の要件

`docs/plans/npr-tone-hatching.md` に以下を含めること。

- **§1 Summary** — 何を足すか、既定 OFF で byte-identical であること、
  既存の `--edges` 2 系統との関係
- **§2 現状の検証済み所見** — `file:line` 付き。この指示書との食い違いを明記
- **§3 データ構造の変更** — `HatchOptions` / `HatchLayer` / `LayerKind` /
  `MarkStyle` / `ToneRecipe` / `GroupHatchStyle` / `FrameResult` 追加 AOV。
  public / internal の別と、`CMakeLists.txt` の `install(FILES)` への追加要否
- **§4 パイプライン改修** — `pipeline.cpp` の正規化ブロック、downsample リスト、
  最終段への `applyHatch` 挿入。それぞれの挿入位置を `file:line` で指定
- **§5 シェーディング改修** — `hit_shader` でのトーン生成
- **§6 `applyHatch` の実装** — 格子 / マーク形状 / 摂動の 3 層分離（§2.4）、
  手続き的 TAM、Lp 面積定数テーブル、50% 反転、レイヤ合成、コントラスト保証、
  TBB による行タイル並列化（既存パスと同じ決定性を保つこと）
- **§6.5 マークスタイルのプリセット表** — §2.4.4 の表を具体値まで埋めたもの。
  実装細部ではなくプロダクト仕様として独立した節にすること
- **§7 CLI** — `--hatch`, `--hatch-layer`, セクション別指定。`--edges` の
  パターンに合わせる。usage 文字列の更新
- **§8 テスト** — 最低限、以下を回帰として押さえること:
  - `hatch.enable = false` で既存出力が byte-identical
  - `applyHatch` を直接叩き、トーンを連続に振ったとき任意のピクセルで
    インクが単調非減少（入れ子性の検証。`firstLevel` のバグを直接捕まえる）
  - ss = 1 と ss = 4 で線幅が同一
  - 4 通りの base/ink 組み合わせすべてでインクが可視（コントラスト保証の検証）
  - `shapeExponent` を 1 / 2 / 4 / 16 と振っても、同一 tone に対する平均被覆率が
    一定（Lp 面積定数補正の検証。ここを外すと形状変更で濃度が変わる）
  - `jitter` / `wobble` を入れてもトーン単調性が保たれる（摂動が tone に
    依存していないことの検証）
  - 被覆率が 50% を超える tone 域で、tone を下げ続けると被覆率が 1.0 に到達する
    （反転ロジックの検証）
  - 決定性: スレッド数を変えて bit-exact
  既存の `tests/test_render_edges.cpp` / `test_image_ops.cpp` の書き方に倣う。
  リファレンス画像には `data/1ab0_scene4_toon1.pov` が使えるはず
- **§9 段階分け** — フェーズごとに「動く絵が出る」単位で区切ること。
  最小の第 1 フェーズを明示する（おそらく: 単一 Line レイヤ・摂動なし・Paper/Fixed のみ・
  セクション別なし・トーンは `N·L × contactAo` のみ。マーク形状と摂動は第 2 フェーズ）
- **§10 非目標** — 物体空間 UV、TAM テクスチャ生成、アニメーションの
  時間的コヒーレンス、GPU 実装、学習ベース手法、
  重み付き Voronoi 等の最適化ベースのスティップル配置（§2.4.1 のジッタで代替する）

作業ツリーに `task lint` は存在しない（build / test / render / deps / clean と
`:static` 版のみ）。検証は `task test` で行うこと。

最後に `docs/plans/README.md` へエントリを追加すること（既存項目と同じ形式・
同じ粒度の要約 + **提案・未着手** ステータス）。

---

## 5. 参考文献

プラン中で手法の出所を示すときに使う。

- Praun, Hoppe, Webb & Finkelstein, *Real-Time Hatching*, SIGGRAPH 2001 —
  TAM の原典。トーン方向・ミップ方向の入れ子性、lapped texture パラメータ化
- Webb, Praun, Finkelstein & Hoppe, *Fine Tone Control in Hardware Hatching*,
  NPAR 2002 — 複数閾値によるブレンドアーティファクト回避。`fade` の元ネタ
- Freudenberg, Masuch & Strothotte, *Real-Time Halftoning: A Primitive for
  Non-Photorealistic Shading*, EGRW 2002 — 閾値テクスチャ方式
- Winkenbach & Salesin, *Computer-Generated Pen-and-Ink Illustration*,
  SIGGRAPH 1994 — prioritized stroke textures
- Tarini, Cignoni & Montani, *Ambient Occlusion and Edge Cueing for Enhancing
  Real Time Molecular Visualization*, TVCG 2006 — 分子では陰影より AO が
  形状知覚に効くという根拠。`ToneRecipe` が AO 中心である理由
- Ostromoukhov & Hersch, *Artistic Screening*, SIGGRAPH 1995 —
  任意形状のスクリーン要素。マーク形状を差し替える設計の先行例
- Secord, *Weighted Voronoi Stippling*, NPAR 2002 /
  Deussen et al., *Floating Points*, NPAR 2000 — スティップル配置の最適化系。
  §2.4.1 のジッタ方式が何を近似しているかの参照先（実装はしない）
