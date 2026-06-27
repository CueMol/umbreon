# umbreon: Fog を POV exp 近似 → OpenGL 線形 fog へ置換 ＋ 透過背景対応 実装プラン

ステータス: **提案・未着手**。本ドキュメントは設計合意の記録。実装は別ツリーの作業完了後に着手すること（今は実装しない）。

調査時点: 2026-06-27。行番号は目安。実装時に各ファイルを再読し、構造的アンカー（関数名/ブロック）を優先すること。

## Context（なぜこの変更を行うか）

現状の umbreon fog（`src/umbreon/render/fog.cpp`）は **CueMol の POV-Ray 出力に書かれた fog を移植**したもの。
しかし CueMol2 の POV fog 自体が、**インタラクティブ表示で使っている OpenGL の線形 fog を POV の指数 fog で近似**したものにすぎない。
そのため umbreon の出力は CueMol2 の画面表示と厳密には一致しない。ユーザが実際に見て調整するのは OpenGL 表示なので、
**OpenGL fog 相当を実装した方が UX として正しい**。

加えて 2 つ目の問題: 背景を `transparent`（透過）にして後から任意の背景画像を重ねたい、という運用ができない。
現状 fog は前景 RGB を fog 色（= 背景色）へ寄せて焼き込むため、透過出力しても遠方ジオメトリに特定の背景色がベイクされ、
後段で背景色を差し替えると遠方が破綻する。

本プランは (A) fog モデルを OpenGL 線形 fog へ置換、(B) 透過背景時に fog 色を焼き込まない alpha フェード方式、の 2 点を扱う。

## 1. リファレンス: CueMol の fog（権威ある定義）

### 1a. OpenGL 線形 fog（画面表示・ターゲット）

`~/proj64/cuemol2/src/sysdep/ogl_core/fog_inc.glsl`:

```glsl
vec4 fragFogColor(in vec4 color, in float frag_alpha, in float fogCoord) {
    float fog = (u_fogEnd - fogCoord) * u_fogScale;   // u_fogScale = 1/(fogEnd - fogStart)
    fog = clamp(fog, 0.0, 1.0);
    float alpha = color.a * frag_alpha;
    vec3 fogmixed = mix(u_fogColor, vec3(color), fog); // = fogColor*(1-fog) + color*fog
    color = vec4(fogmixed, alpha);                     // ← alpha は fog の影響を受けない
    return color;
}
float ffog(in float ecDistance) { return abs(ecDistance); }  // fog座標 = |eye-space z|（平面、放射距離ではない）
```

要点（POV exp 移植との本質的な違い）:
- **線形**: `f = clamp((end - z)/(end - start), 0, 1)`。`f=1` で近（素の色）、`f=0` で遠（fog 色）。
- **fog 座標は平面 eye-z**（`|z_eye|`）。放射距離（ray の長さ）ではない。透視投影で両者は異なる。
- **alpha は fog で変化しない**。RGB のみ fog 色へ寄る。背景は fog 色（= 背景色）でクリアされるため、遠方が背景へ溶ける。

`fog_start = fognear`, `fog_end = fogfar`, `fog color = 背景色` は `~/proj64/cuemol2/src/qsys/GUIView.cpp:93-100, 494-502`:

```cpp
double fognear = dist;                 // dist = getViewDist()（カメラ→注視点距離）
double fogfar  = dist + slabdepth/2.0;
if (fognear < 1.0) fognear = 1.0;
pdc->setFogStart(fognear);  pdc->setFogEnd(fogfar);
// setFogColorImpl → fog 色 = 背景色
```

### 1b. POV 出力（現 umbreon の移植元）

`~/proj64/cuemol2/src/modules/rendering/PovDisplayContext.cpp:373-382`:

```cpp
fog {
  distance <m_dSlabDepth>/3        // ← POV distance = slabDepth / 3
  color rgbf <bg.r, bg.g, bg.b, 0> // ← fog 色 = 背景色
  fog_type 2  fog_offset 0  fog_alt 1.0e-10  up <0,0,1>   // 線形 fog を ground-fog ハックで近似
}
```

`fog_type 2 / fog_alt≈0` は CueMol が POV 上で線形っぽい減衰を作るためのハック。**この type/offset/alt/up は OpenGL fog には存在しない**。
新モデルではこれらを使わず、`distance` と `_distance` から線形 fog パラメータを復元する（§3）。

## 2. 現 umbreon 実装の分析

- `src/umbreon/render/fog.cpp` `fogTransmittance()`: `exp(-fogLength/distance)`（指数）。`type==2` で `up`/`offset` を使った ground fog 近似。
  fog 座標は**単一の `camDir` に沿った深度**で、ピクセルごとの ray 方向を使っていない（透視で更に誤差）。
- `applyFog()`: 背景（`!isfinite(d) || d>=maxDepth`）をスキップし、ヒット画素の RGB のみ fog 色へブレンド（alpha 不変）。
  supersample 解像度で downsample 前に適用（`src/umbreon/umbreon.cpp:90-93`）。
- `Fog` 構造体（`src/umbreon/scene.hpp:246-254`）: `enabled / color / distance / type / offset / alt / up`。
- POV パース（`src/bench/pov/pov_scene_reader.cpp:539-` `parseFog()`）: 上記フィールドへ素直に格納。
- 深度 AOV: `pr.depth = nearDepth = rh.ray.tfar`（**放射距離**、rd は単位ベクトル）。
  平面 eye-z は `pr.viewZ = tfar * dot(rd, camDir)`（`transparency.hpp`）だが、`res.viewZ` は **edges 有効時のみ確保**（`embree_renderer.cpp:264-270`）。
- 透過背景: `opt.transparentBackground` 時 `baseCov = 0`（`transparency.hpp:78`）で背景 alpha=0。`RenderOptions` 経由で `umbreon::render()` から参照可能。

## 3. POV → OpenGL fog パラメータ復元

POV ファイルから線形 fog の `start/end/color` を一意に復元できる:

| OpenGL fog | 復元式 | 出所 |
|---|---|---|
| `fogColor` | POV fog の `color`（= 背景色） | そのまま |
| `slabDepth` | `3 * fog.distance` | POV `distance = slabDepth/3` の逆算 |
| `fogStart`  | `_distance`（POV ヘッダの宣言。なければ `max(1.0, _distance)`） | CueMol `fognear = dist`、`_distance` は view 距離 |
| `fogEnd`    | `fogStart + slabDepth/2 = _distance + 1.5 * fog.distance` | CueMol `fogfar = dist + slabDepth/2` |

`_distance` は POV ヘッダで `#declare _distance=...` 済み（lights パースが既に `symScalar("_distance", 200.0)` で参照）。
フォールバック: `_distance` 不在時は `|look_at - location|`、それも無ければ現行どおり。`fognear` の `max(1.0,...)` クランプも踏襲。

→ **POV リーダ（`parseFog` または `finalize`）で `Fog.start / Fog.end` を計算して格納**し、レンダラ/fog.cpp は復元ロジックを持たない。

## 4. 設計

### 4a. fog 係数（両モード共通）

```
z   = 平面 eye-z = |dot(P - camPos, camDir)|     // = 既存 viewZ
f   = clamp((fogEnd - z) / (fogEnd - fogStart), 0, 1)
```

`fogEnd <= fogStart` の縮退時は `f = (z < fogEnd ? 1 : 0)` 等でゼロ除算回避。

### 4b. 不透明背景（既定）= CueMol GL 完全一致

```
RGB = mix(fogColor, objRGB, f) = fogColor*(1-f) + objRGB*f
alpha は不変
```

背景画素は fogColor = 背景色のまま（現行どおりスキップで可。GL も背景を fog 色でクリアするため一致）。

### 4c. 透過背景 = alpha フェード（fog 色を焼き込まない）

透過運用では fog 色を RGB に焼き込まず、**fog 係数で alpha を減衰**させる:

```
RGB   そのまま（fog 色へ寄せない）
alpha = alpha * f
```

正当性（ストレート alpha 出力を後段で背景 B に "over" 合成）:
```
result = objRGB*(alpha*f) + B*(1 - alpha*f)
       = objRGB*f + B*(1-f)          // 不透明ヒット alpha=1 の場合
```
これは GL の `fogColor*(1-f) + objRGB*f` と **B = fogColor のとき厳密一致**し、任意の別背景 B' でも「遠方が B' へ溶ける」物理的に正しい結果になる。
→ 透過出力 → 後から背景差し替え、が破綻しない。fogColor は透過モードでは不要（焼き込まない＝差し替え自由）になる。

注: 半透明オブジェクト（intrinsic alpha<1）でも `alpha*=f` は被覆率を一様にスケールするので破綻しない。
本方式は不透明ヒットで GL と厳密一致、半透明では premultiplied 相当の自然な一般化。

## 5. 実装ステップ（ファイル別）

1. **`src/umbreon/scene.hpp` `Fog`**: `float start`, `float end` を追加。`distance/type/offset/alt/up` はレガシー POV 互換用に残す（任意）。
2. **`src/bench/pov/pov_scene_reader.cpp`**: `_distance` を保持し、§3 の式で `fog_.start / fog_.end` を計算・格納。`color` は現行どおり背景色。
3. **`src/umbreon/render/fog.{hpp,cpp}`**:
   - `fogTransmittance` を線形係数 `f`（§4a）へ置換。引数は平面 eye-z。
   - `applyFog` に `bool transparentBackground` と平面深度 AOV を渡す。`transparentBackground` で §4c（alpha 減衰）、否なら §4b（RGB mix）。
   - 平面 eye-z の入手は下記いずれか:
     - (推奨) `embree_renderer.cpp` の `res.viewZ` 確保ゲートを `opt.strokeEdges.enable || (fog 有効)` へ拡張し、`applyFog` は `viewZ` を読む（既存 AOV 再利用、ray 生成の重複なし）。
     - (代替) `applyFog` 内でピクセル毎に ray 方向を再構成し `z = depth * dot(rd, camDir)`。ray 生成式の重複に注意（共有ヘルパ化が必要）。
   - 旧 `type/offset/alt/up` の ground-fog 経路は削除 or `--fog-model=pov` レガシー分岐に隔離。
4. **`src/umbreon/umbreon.cpp:90-93`**: `applyFog(...)` 呼び出しを新シグネチャへ（`opt.transparentBackground` と viewZ を渡す）。supersample → downsample 前適用は踏襲。
5. **`src/bench/main.cpp:222-228, 507-509`**: fog ログを `start/end`（または復元値）表示へ更新。

## 6. 既定挙動・互換性

- **既定の fog は線形へ切替**（これが改善本体）。⇒ **fog を含む canonical 出力は bit-exact が崩れる**。再生成＋視覚再承認が必要。**fog 無しシーンは無変更**。
- 透過背景＋fog は新規挙動（従来は実質未対応）。`--transparent-bg`（既存 `transparentBackground`）と fog の組合せで §4c が効く。
- レガシー POV exp fog が必要なら `--fog-model={gl|pov}`（既定 `gl`）で旧経路を温存（回帰比較用、任意）。

## 7. CLI / パラメータ

- 既定は POV から復元（§3）で追加フラグ不要。
- 任意で上書きフラグ: `--fog-start`, `--fog-end`, `--fog-color`, `--no-fog`（既存の `_no_fog` declare ルートと整合させる）。
- `--fog-model=gl|pov`（任意、レガシー）。

## 8. テスト

- **解析テスト**（`tests/test_render.cpp` 系）: 既知 `start/end/z` に対し線形係数 `f` と合成結果を検証（近=素色、遠=fog 色/alpha、中間=線形補間）。平面 eye-z（透視で放射距離と異なる）を 1 ケース含める。
- **透過合成の往復テスト**: 透過＋fog 出力を背景 B=fogColor へ "over" 合成 → 不透明背景レンダリングと一致（§4c の厳密一致を保証）。別背景 B' でも数式どおりを確認。
- **POV 復元テスト**: `_distance` と `distance` から `start/end` が式どおり復元されること（`pov_scene_reader` 単体）。
- **視覚確認**: fog ありシーンで CueMol2 画面と勾配が一致することを目視。
- fog 無し経路の bit-exact 維持を回帰で担保。

## 9. 未決事項（実装前に確認）

- 平面 eye-z の入手は §5-3 の (推奨) viewZ 常時確保か (代替) ray 再構成か。メモリ vs 重複のトレードオフ。→ 既存 viewZ 再利用が低リスク。
- レガシー POV exp 経路（`--fog-model=pov`）を残すか破棄するか。回帰比較の必要性しだい。
- `fogStart` 復元で `_distance` 不在時のフォールバック優先順位（`|look_at-location|` か固定値か）。
