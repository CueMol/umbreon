# 半透明 veil の輪郭線を per-pixel 合成で残す

半透明 (group alpha) のセクションに stroke edge (`--edges`) を付けると、per-pixel の
group blend (`GroupBlendMode::PerPixel`) で外周の輪郭線が消え、セクションの内側の線だけが
残る。veil のパスが描いたインクも「veil が覆う画素」として合成に含め、輪郭線を veil と
同じ alpha で残す。

対象は umbreon。cuemol2 は umbreon を更新したあと、`build_scripts/deplibs.env` の
`UMBREON_GIT_REF` を新しいコミットに移すだけで、cuemol2 側のコード変更は無い。

## 1. 原因

- group blend は、veil をすべて隠した背景のパスと、veil ごとのパスを別々にレンダリングする
  (`blend/group_blend.cpp`)。stroke edge は合成の前、各パスの `renderFrameRaw()` の中で
  そのパスの色に焼き込まれる (`render/pipeline.cpp` → `applyStrokeEdges`)。veil の
  輪郭線は veil のパスの色にしか無い。
- `blendPerPixel()` が veil のパスの色を使うのは `veilCovers(dBg, dVeil)` が真の
  サンプルだけ。判定は光線の当たった距離 (`FrameResult::depth`) だけを見て、インクを
  見ない。
- stroke の既定の位置は `StrokeAlign::Outside` で、遮蔽輪郭 (Silhouette / ObjectId /
  DepthGap) の幅を全部、輪郭の外側 (背景・奥の物体の側) に乗せる。外周の輪郭線の画素は
  veil に当たっておらず (`dVeil = 0`、または背景と同じ距離)、合成で捨てられる。
- 残るのは外側も同じ veil の上にある線 (veil が自分に重なる折り目など) だけになる。
- LayerWeights はパス全体の色を重みで混ぜるので、この問題は出ない (輪郭線は重み a で残る)。

## 2. 方式

### 2.1 veil のインクの被覆を記録する

- `FrameResult` に、サンプルごとのインクの被覆 `vector<float> inkCover`
  (width*height、0..1) を足す。確保するのは要求されたときだけ (下記)。既定の経路
  (group blend なし、edge なし) では確保せず、出力は byte-identical のまま。
- 記録の対象は、**そのパスの veil に属するセクションが持ち主の run** だけにする。
  - 背景にもある不透明なセクションの線 (veil のパスでも同じように描かれる) を被覆に
    含めると、そのサンプルが veil に覆われたことになる。veil が 2 枚以上あると、背景の
    重み `T = Π(1 - a_i)` と veil どうしの配分がずれる (本物の veil の色と、背景と同じ
    色が `a_i` の比で混ざる)。持ち主で絞ればこれが起きない。
  - run の持ち主は `StrokeChainInput::group`。screen source では edge group
    (`Scene::edgeGroupOfGroup` で写したもの) なので、veil のグループも edge group に
    写して比べる。veil のグループと不透明なグループが同じ edge group に入っている
    場合は、その edge group の run を veil のものとして扱う (edge group は「1 つの
    セクション」として描く指定なので、これで整合する)。
- 描画区間の結合を veil の所属で切る: screen source は、隣り合う run の線のスタイル
  (幅・色・不透明度)・外側の向き・深度の連続が揃うと、edge group が違っても 1 本の
  描画区間にまとめ、先頭の run の group を `StrokeChainInput::group` に入れる
  (`screen_vector_edges.cpp` の DRAW-SPAN coalescing)。veil と不透明なセクションが
  一続きの外周を作り、両方が同じ線のスタイルだと (cuemol2 の既定ではよくある)、
  先頭が不透明側の区間では veil 側の外周線まで記録から漏れる。インクを記録する
  パスでは、結合の条件に「両方の run の veil の所属が同じ」を加え、所属が変わる所で
  区間を分ける。
  - 区間ごとに所属が 1 つに決まるので、塗りの側は区間単位の判定のままで済む
    (点ごとに所属を持たせて塗りで補間する案は、塗りの関数を全部変えることになり
    採らない)。
  - 分けた所では 2 本の線の端が突き合わせになり、曲がり角の外側に 1 px 程度の
    隙間が出得る (結合の目的だった隙間)。出るのはインクを記録する veil のパスの、
    veil と不透明なセクションの境目だけで、不透明側の線は背景のパスにも描かれて
    いる。目視で確かめる (§4)。
  - インクを記録しないパス (背景のパス、group blend なしの描画) では条件を足さず、
    今の出力のまま。
- 指定の渡し方: `RenderOptions` には足さず、group blend が `renderFrameRaw` に内部の
  引数で「インクを記録する edge group の集合」を渡す。渡すのは **PerPixel で stroke
  edge が有効なときの veil のパスだけ** (`blendPerPixel()` の veil の呼び出しが
  `PassRunner::run` に集合を渡す)。背景のパス、LayerWeights のパス (同じ
  `PassRunner::run` を通る)、edge が無効なときは渡さず、`inkCover` も確保しない。
  LayerWeights の描画区間の結合も今のまま変わらない (§4 の比較の基準になる)。
- 記録の仕方: `stroke_render.cpp` の塗り (`fillTriangle` / `fillTriangleAlpha`、
  `compositeOver` を呼ぶ所) で、対象の run の塗りのときだけ
  `inkCover = inkCover + a * (1 - inkCover)` で積む (`compositeOver` の over と同じ
  式で、重ね塗りしても 1 を超えない)。debug 用の塗り (`fillDisc` など) は対象外。

### 2.2 合成で被覆に加える

- `blendPerPixel()` の被覆の判定を「面が覆う、または veil のインクがある」にする:
  `veilCovers(dBg, dVeil) || (!inkCover.empty() && inkCover[p] > 0)`。重みは今と同じ
  `v.alpha`。空の `inkCover` (edge が無効、§2.1 で確保しない場合) は「インクの被覆
  なし」で、判定は今の `veilCovers` だけと同じになる (edge なしの既存テスト P2 が
  この経路を通る)。
- インクだけのサンプルでも、veil のパスの色をそのまま使う。veil のパスの色は
  `(1 - k) U + k I` (U はインクの下の色、I はインクの色、k はインクの不透明度)。
  U が背景のパスの色 B と同じなら、合成は `(1 - a) B + a ((1 - k) B + k I)
  = (1 - a k) B + a k I` で、インクを不透明度 `a * k` で B に重ねたものと一致する。
  インクの色を別に持つ必要はない。
  - U と B が違うのは、インクの下が背景のパスと違う描かれ方をしている所だけ
    (veil のパスでは veil も遮蔽物として影・AO を落とす、背景のパスにしかない線が
    ある、など)。その差がインクの幅の範囲で `a (1 - k)` だけ混ざる。インクは
    ハードに塗る (サンプル単位では k はほぼ線の不透明度) ので、実用上は目立たない
    見込み。目視で確かめる (§4)。
- veil が 1 枚なら、結果は LayerWeights のインクの残り方と同じ (輪郭線の画素で
  `(1 - a) B + a S`)。

### 2.3 変えないもの

- LayerWeights の合成、edge の抽出・分類、stroke の位置 (`StrokeAlign`)、fog の
  インクへの掛け方。
- veil の面の画素の扱い (今まで通り、面が覆うサンプルは veil のパスの色)。
- 透明背景での stroke の alpha: `compositeOver` は alpha を変えないので、透明背景では
  背景の上のインクの alpha が 0 のまま残る。これは veil に限らない既存の挙動で、
  この plan では扱わない (§4 の確認で問題が見えたら別の作業にする)。

## 3. テスト

`tests/test_render_transparency.cpp` (veil の合成の既存テスト T9 / T10 の並び) に 1 件:

- 不透明な背景の板の手前に、alpha 0.6 の veil (板より小さい quad) を置き、stroke edge を
  有効にして per-pixel でレンダリングする。veil の外周のすぐ外側 (`Outside` のインクが
  乗る画素、veil の面には当たっていない) が、背景の色と線の色を 0.4 : 0.6 で混ぜた色に
  なることを確かめる。今のコードでは背景の色のまま (インクが消える) なので、この
  不具合の再発を検知できる。
- 同じテストのシーンに、veil の横に接して、同じ線のスタイルの不透明な板を置く。
  2 枚の共通の外周に沿って、不透明側から veil 側へ移ったあとの veil の外周線の画素も
  同じ色になることを確かめる (§2.1 の描画区間の分け方。分けないと、先頭が不透明側の
  区間で veil 側の線が消える)。シーンを 1 つ増やさず、同じテストの判定を 1 つ足す。

不透明なセクションの線を被覆に含めない (§2.1) ことのテストは足さない。veil 2 枚の
重なりの検算は既存の T9 / T10 が担い、そちらは edge なしで変わらない。

## 4. 進め方

1. umbreon でブランチを作り、§2.1・§2.2 を実装する。
2. umbreon_cli で、半透明の surface に edge を付けたシーンを per-pixel と LayerWeights で
   レンダリングし、外周の輪郭線が両方で同じように (alpha 0.6 で) 出ることを確かめる。
   cuemol2 の `~/Desktop/veil.qsc` を umbreon にエクスポートしたものを使う。
   veil と不透明な板が接するシーンでも、共通の外周の線が veil 側で途切れないこと、
   区間を分けた境目に目立つ隙間が出ないことを見る。
3. cuemol2 で `task install_umbreon` してアプリから `veil.qsc` をレンダリングし、目視で
   確認してもらう。輪郭線の濃さ (veil と同じ alpha) をここで決める。
4. 確定後に §3 のテストを足す。umbreon の PR を出し、マージコミットでマージする。
5. cuemol2 で `UMBREON_GIT_REF` を umbreon のマージ後のコミットに移す PR を出す
   (この PR の CI が組み合わせを検証する)。

## 5. 決めたこと

- 輪郭線は veil と同じ alpha で背景に混ぜる (方式 A)。不透明のまま残す案、および
  edge を合成のあとに 1 回だけ描く案 (背景と veil の G-buffer をまとめて抽出し直す)
  は採らない。後者は edge の分類を作り直すことになり、変更が大きい。
