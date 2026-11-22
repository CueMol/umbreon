# Object-space edge 計画の第1段階レビュー

第1段階は、不透明なシーンで CueMol から object 方式を選び、renderer ごとの設定どおりに輪郭を描ければ完了とする。この範囲で実装に進める。

## 1. 曲線で描く輪の完了条件を §2.1 と §4 に反映する

現行の §6 では輪の描き方は未決定。Bézier 曲線を採用する場合は、§2.1 の描画方針と §4 の「分割点と弦の中央」の試験を曲線に合わせる。単独の球では、円形断面の3次 Bézier 曲線8本で輪を描き、中心線を一周する3,600本の光線がすべて命中することを確認済み。組み込み時は、stick との接合部で必要な区間だけが切れ、残した円弧と stick の側線が斜めから見ても隙間なくつながることを確認したい。[計画 §6](edge-object-space-completion.md#L451)・[計画 §4](edge-object-space-completion.md#L405)・[単独の球での試験結果](edge-object-space-completion-review.md#L31)

## 2. 接合部の回帰テストでは球と stick を同じセクションに置く

§2.4 は同じセクションの球・stick に対して接合部を切る。一方、§4 が参照する既存テストは、生成した輪郭を数え分けるために球を `group=1`、stick を `group=2` にしている。この配置に同一セクション条件を適用すると、同じ半径の stick が球の輪郭を切るというチェックが失敗する。両方を同じセクションに置き、球単独を基準に、細い stick では輪が一周残り、球と同じ半径の stick では接合側の円弧だけが切れることを確認したい。この期待値は輪の描画方式によらず使える。[計画 §2.4](edge-object-space-completion.md#L254)・[計画 §4](edge-object-space-completion.md#L374)・[既存テスト](../../tests/test_object_space_edges.cpp#L348)

## 3. CueMol の目視確認で renderer ごとの色・太さ・無効指定を見る

§4 の単体テストは Umbreon のスタイル表を直接設定する。CueMol exporter からの受け渡しは、§3 で予定している不透明なシーンとカメラで確認できる。画面に見えている2つの renderer に異なる `egcolor` と `eglinew` を指定し、object 方式の画像で両者を見分けられることを確かめる。次に片方だけを `egtype=none` にして、その renderer の輪郭だけが消えれば、無効指定まで含めて確認できる。[第1段階の目視確認](edge-object-space-completion.md#L342)・[スタイル表の単体テスト](edge-object-space-completion.md#L380)
