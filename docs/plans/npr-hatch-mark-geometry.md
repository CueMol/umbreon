# Tone Hatching: マーク形状と shading の分離（dotScale / Stipple / coverage テーブル / auto fade / strength・curve）

*Plan location:* `docs/plans/npr-hatch-mark-geometry.md`
*Status:* 実装済み（umbreon 0.2.0）。`npr-tone-hatching.md` の Phase 1-3 の上に積む変更。
*Host:* CueMol2 tritium の umbreon (NPR) backend（`docs/architecture/umbreon-hatch-layer-editor.md` 側）。

---

## 1. 動機（cuemol2 からの報告）

1. GUI の `Mark width`（`hatchWidthScale`）が manga / screentone-60 / manga-square / stipple で効かない。
   `HatchLayer::widthPx` は Line 専用で、Dot の半径は `step * rho_p * sqrt(1 - tone)` と
   **pitch と tone だけ**から決まり、ドット径の knob 自体が無かった。
2. stipple（Dot, K=2）は最細セル基準の面積正規化 + nesting のため、tone 0.35 より明るい大半で
   被覆が本来の 1/16〜1/4 しかなく、0.35 → 0.27 で急に暗転する。
3. Line は幅こそ自由だが `widthPx <= 2*step` で頭打ち（pencil ≈1.25x）、tone 応答は nesting の
   段階挿入 + 狭い fade 帯で階段状。
4. 既存の 50% 反転（凍結ドット + 双対格子の白穴の `max` 合成）は、穴が十分縮むまで平均被覆が
   **0.5 で平坦**（表示 tone ≈0.5 → 0.3）になり、その後急上昇する。screentone の中間調が shading に
   追従しない。
5. richardson 以外の全 style は平坦な既定 `ToneRecipe`（wrap 0 / rim 0 / gamma 1）のまま。
   CueMol のフラッシュ光ではほぼ全面が tone ≈ 0.9 に集中し、default が薄い主因。

ユーザー要件: **マーク形状（太さ・径・密度）を tone から独立**させ、被覆による暗さは
**面の照光の明るさに連続・単調に相関**させる。太くして黒潰れするのは許容。

## 2. 決定事項

| 項目 | 決定 |
|---|---|
| マーク寸法 | `HatchLayer::dotScale`（無次元、既定 1、ss 変換なし）。Line は `widthPx`、Dot / Stipple は `dotScale` |
| Dot の gain | 半径と（旧）反転判定が見る tone を `t_eff = clamp01(1 - dotScale^2 (1 - t))` に置換。閾値と fade は元の `t`（密度・nesting 不変） |
| Dot の被覆 → 半径 | 双対格子の穴を廃止。`hatchNormalizeLayer` で **被覆 → 半径テーブル**（layer 自身の mark 関数で 1 セルの平均被覆を半径 64 段 × 16^2 点で標本化し逆引き）を作り、`hatchDotInk` は目標被覆 `1 - t_eff` の半径を引く。非重畳域では面積正規化の閉形式と一致し、重畳域も覆域半径（+AA）まで連続 → K=0 は全域で `被覆 = 1 - tone` |
| `invertAbove50` | 意味を「覆域半径まで成長して真っ黒に到達」に読み替え（off は面積正規化の全セル半径で停止、被覆 ≈0.9） |
| Stipple | 新 `LayerKind::Stipple`（=2）。セル毎 hash 閾値 `uT = toneLo + (toneHi - toneLo) u` で `tone < uT` のセルだけ、固定半径 `dotScale * step * rho_p` のドット（出現閾値直下 `1/fadeInv` で fade in）。常に union 合成。期待被覆 `dotScale^2 (toneHi - tone)/(toneHi - toneLo)`。subdiv・反転は無視 |
| 幅クランプ | `halfWidth = 0.5 * min(widthPx, max(spacing, 2*step))`（level-0 pitch でベタ。K=0 では旧 `2*step` の方が広く、fade 途中では意味があるので旧上限を下回らない）。`padLine` は追随済み |
| auto fade | `fadeInv <= 0` = 出現閾値から**次の level の閾値**（K=0 は toneLo）まで線形に 0 → 全幅（`fadeInv = K / (toneHi - toneLo)`）。Line / Dot の被覆を tone の連続関数にする。Stipple では 32 と同義 |
| richardson look | 調整済みの look なので pencil preset の auto fade は継承せず `fadeInv = 10` を明示。変更前ビルドとレンダーが byte 一致することを CLI で確認済み |
| tone → coverage | `ToneRecipe::strength`（線形 gain）と `ToneRecipe::curve`（被覆域指数）。display encode と highlight knee の**後**、levels 量子化の**前**で `c = clamp01(strength (1 - t)^curve); t' = 1 - c`。既定 (1,1) は表示域線形 = 古典網点。既存 `gamma`（リニア域・whitePoint と結合）は流用しない |
| preset の tone recipe | `hatchPresetTone(name, ToneRecipe&)` テーブル + `applyHatchStyle(opt, name)`（look なら look、さもなくば preset + テーブル）。初期値は全 preset とも `richardsonTone()`（look の SSOT。**TODO(phase4)**: mark 種別ごとに目視調整）。`ink-cross` / `manga` look も下敷き preset のテーブルを使う（look = preset + 紙/インクモデル） |
| spec テキスト | `applyHatchLayerKv` / `applyHatchToneKv` / `applyHatchInkKv` / `applyHatchSpec` / `hatchStyleToSpec` を `npr/hatch_shade.hpp` に公開。CLI の `--hatch-layer` / `--hatch-tone` も同じ語彙を使う。ホストはこれで style をテンプレートとして読み書きする |

## 3. 数式と不変量

### 3.1 Dot gain と被覆テーブル

- `t_eff = clamp01(1 - s^2 (1 - t))`（s = dotScale、s = 1 なら bit 同一の `t`）。
- `r = rTab(1 - t_eff) * fade(level, t)`。`rTab` は非減少、`fade` は t の非増加関数、位置は hash のみ
  → 画素単位で暗くなる方向に単調、マークは動かない・消えない。
- テーブル: 半径 `q/64 * rMax`（`rMax = 覆域半径 2^(1/p)(0.5+jitter)·aspect·step + halfAA`、
  `invertAbove50` off なら `step·rho_p`）ごとに 1 セル 16×16 点で `max`（4×4 近傍のドット）を平均。
  逆引きは線形補間、到達不能な被覆は `rMax` に飽和。jitter は無視（近似）。
- s > 1 は `t = 1 - 1/s^2` で既に真っ黒（許容）、s < 1 は最大被覆 ≈ s^2。

### 3.2 Stipple

`E[c] = s^2 · clamp01((toneHi - t)/(toneHi - toneLo))`（重なり分だけ減る）。`toneHi == toneLo` は
「toneHi 未満で全セル出現」のステップ。tone = 1 では `u < 1` なので常に不在（紙は無インク）。

### 3.3 auto fade の被覆形状（pen-cross, strength = curve = 1）

L0 は表示 tone [0.95 → 0.75 → 0.55 → 0.35] で 0 → 0.11 → 0.22 → 0.44 の区分線形、L1 / L2 も同形。
乗算合成後は概ね 0.03@0.9, 0.11@0.75, 0.2@0.6, 0.35@0.5, 0.5@0.4, 0.65@0.3, 0.8@0.2（ペン画らしく
ベタにはならない）。

### 3.4 tone チェーン（`applyHatch`）

```
t = clamp01((tLin - black)/(white - black)) -> pow(t, gamma) -> srgbEncodeF
-> highlight knee -> c = clamp01(strength * (1 - t)^curve); t = 1 - c -> levels 量子化
-> 各 layer 評価 / inkShadeDark
```
`strength` は `max(0, ·)`、`curve` は `[0.1, 10]` にクランプ。

## 4. spec テキスト文法

```
spec    := line ( ("\n" | ";") line )*
line    := ws* ( <empty> | "#" comment | ("layer" | "tone" | "ink") ws* ":" entries )
entries := [ key "=" value ( "," key "=" value )* ]
value   := float (C locale) | int | on|off (1/0/true/false も可) | enum word | "#rrggbb"
```
- `layer:` 行は 1 行 = 1 layer。1 行以上あれば `layers` を**全置換**、0 行なら不変。
- `tone:` / `ink:` はキー単位の上書き。エラー時は対象不変で `"line N: ..."`。
- キー: layer = `kind angle spacing subdiv width dotscale tonehi tonelo fade opacity inkscale soft seed shape aspect dotangle jitter invert wobble wobwave wjitter slen sgap taper anglejitter lenjitter tooth toothscale`、
  tone = `diffuse ambient wrap rim rimpow rimbias contact shape black white hl hlsoft gamma speccut strength curve levels`、
  ink = `mode base ink inkcolor papercolor mincontrast inkshade tonefog albedoquant`。
- シリアライズは layer 行（kind でキーをフィルタ）→ `tone:` → `ink:`、数値は classic locale の
  `setprecision(6)`、色は `#rrggbb`。往復でテキスト一致（テスト §27）。

## 5. 互換性

- `HatchLayer` / `ToneRecipe` のレイアウト変更 → 利用側は全再ビルド。バージョン 0.1.0 → 0.2.0。
- `applyHatchPreset` は従来どおり layer のみ。ただし pen-cross / pencil / engraving は `fadeInv = 0`
  （auto）、stipple は Stipple 種別に変わり、Dot の反転はテーブル方式になったので、**preset 指定の絵は
  意図的に変わる**。既定構築の `HatchOptions`（`enable = false`）は従来どおり byte-identical。
- CLI: `--hatch-width` は Line のみ、Dot / Stipple は `--hatch-dot-scale`。look 無しの `--hatch-preset` /
  bare `--hatch on` は `applyHatchStyle`（preset の tone recipe 付き）。新フラグ `--hatch-spec` /
  `--hatch-dump-spec`。

## 6. テスト（`tests/test_hatch.cpp` §23-29）

dotScale の被覆スケーリング・反転込み単調性・真っ黒到達 / K=0 スクリーンの重畳域 tone 追従 /
Stipple の tone 追従・紙無インク・subdiv 無視・単調 / 幅クランプ緩和 / auto fade の連続性 /
spec 往復・エラー拒否・section mask・kv / strength・curve / `applyHatchStyle` と `applyHatchPreset` の分離。
