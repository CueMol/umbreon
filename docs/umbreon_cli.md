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
| `--ao-distance` | シーン依存（下記） | AO の届く半径（world 単位）。未指定だと auto = 0.7×bbox 対角＝大局的 |
| `--ao-intensity` | 1.0（full）/ 控えめは 0.6–0.8 | AO の強さ |
| `--shadows` | on | 影を有効化 |
| `--light-radius` | 2–6 度（自然なソフト影） | エリアライトの角半径（度・シーン非依存）。0＝ハード影。大きいほど penumbra が広くノイジー |
| `--shadow-samples` | 16–32 | ソフト影のサンプル数（>1 でソフト化）。light-radius を上げたら増やす |

## `--ao-distance` はシーンに合わせる（鍵）

- 未指定 = auto（0.7×bbox 対角）→ 大局的でぼんやり（scene6 では 54）。
- 窪み・ポケットを際立たせるには小さめに：分子サイズのおおよそ 1〜2 割。
  scene6（対角 ~77）なら 12〜20。
- 起動時ログに auto 値（`ambient occlusion: ... radius XX`）が出るので、それを目安に調整する。

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
