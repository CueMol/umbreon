# 半透明 veil の輪郭線を残す計画のレビュー

veil の外周線を面と同じ `v.alpha` で PerPixel 合成する方針は妥当です。前回指摘した、同じ線スタイルの不透明面と veil が外周を共有するケースは、改訂版の §2.1 が所属の境目で描画区間を分け、§3 が veil 側の外周の画素を確認する形で対応しています。通常の 1〜2 枚の veil を対象に、この方針で実装に進めます。[対象の計画書](edge-veil-ink-coverage.md)

1. **§2.2 で、空の `inkCover` は「線の被覆なし」と明記してください。** §2.1 では被覆バッファの確保は任意です。edge 無効で PerPixel 合成したとき、veil の面の外では `veilCovers(...)` が偽になるため、記載どおりの式は空の `inkCover[p]` を参照します。`veilCovers(...) || (!inkCover.empty() && inkCover[p] > 0)` で両方の経路を扱えます。既存の P2 に、edge 無効で左右の veil の間に背景が見えるシーンがあり、この条件を通ります。[合成ループ](../../src/umbreon/blend/group_blend.cpp#L315-L319) · [既存の P2](../../tests/test_render_transparency.cpp#L325-L341)

2. **§2.1 の記録対象を「PerPixel で edge が有効な veil パス」と指定してください。** `PassRunner::run` は LayerWeights の veil パスでも使われます。一律に「veil パスなら記録する」と、§4 で比較に使う LayerWeights 側まで描画区間が分かれ、接合部の線の形が変わり得ます。`blendPerPixel()` の veil 呼び出しだけが記録対象の集合を渡せば、比較に使う従来の描画を保てます。[LayerWeights の呼び出し](../../src/umbreon/blend/group_blend.cpp#L231-L251) · [PerPixel の呼び出し](../../src/umbreon/blend/group_blend.cpp#L307-L311)
