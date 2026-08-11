# 修正プラン: known-issues 6件(レンダー例外時リーク / GI shadow 不整合 ほか)

対象ブランチ: `fix/known-issues-render-perf-and-leaks`(main = `63e7c52` から分岐)。
このドキュメントは [known-issues-render-perf-and-leaks.md](known-issues-render-perf-and-leaks.md)
に記録された問題1〜6の修正プラン。この1枚で別セッションでも実装を再開できる自己完結を意図する。
行番号は 63e7c52 時点の目安(ずれたらシンボルで grep)。

---

## 1. 背景(なぜこの変更か)

CueMol3 の「GI レンダー後に遅くなったまま戻らない」報告の調査で、症状本体
(Electron renderer プロセスの darwinbg 固着と特定済み。cuemol2 側
`docs/architecture/umbreon-render-qos-throttling.md` 参照)とは独立に実在する
umbreon 側のバグ・問題点が6件見つかった(詳細は known-issues 文書)。本プランは
その6件を修正する。darwinbg 対策自体はスコープ外(cuemol2 側で対応)。

決定事項:

- 全6件を本プランで対応。実装は下記の6コミットに分割する。
- 問題2は「`shadows` 設定を GI にも適用する」。ただし pt1 は凍結アンカー
  (`render_options.hpp:124-127`「bit-identical forever」)、cache(giIntegrator==0)も
  frozen のため、適用は **pt2(`giIntegrator==2`、cuemol2 の GI ON 既定)限定**とし、
  pt1/cache は現挙動(常時 shadow-correct)を維持する。
- 補足(調査で確定): 見た目の落ち影は直接光パスが描いており shadow 設定に従う。
  問題2の実害は「GI gather 内部のシャドウレイ(コスト)が shadow OFF でも止まらない」こと。

## 2. コミット分割と順序(設計決定)

原則: **画像が変わらない変更を先に全部入れ、画像が変わる問題2(C5)を最後から2番目に
1回だけ入れる**。refactor_check のベースライン更新が1回で済み、C1〜C4 は既存
ベースラインで bit-exact 検証できる。

| # | 内容 | 画像への影響 |
|---|---|---|
| C0 | 本プラン文書 + known-issues 文書の追加 | なし |
| C1 | 問題1: 所有権移譲の前倒し + scene_build の try/catch | なし(bit-exact) |
| C2 | 問題4: OIDN `setAffinity=false` | なし(要実測確認) |
| C3 | 問題3: `giWriteAov` フラグで GI デバッグ AOV をゲート | color は bit-exact |
| C4 | 問題5: エッジパスのキャンセル配線 | なし(bit-exact) |
| C5 | 問題2: pt2 gather NEE の shadows ゲート | **pt2 系のみ変わる(意図的)** |
| C6 | 問題6: コメント/ドキュメントのみ + known-issues ステータス更新 | なし |

C1〜C4 は互いに独立。C6 は「実害経路は C5 で閉じた」ことを根拠に書くので C5 の後。

## 3. C1: 問題1 — 例外時の RTCDevice/RTCScene リーク

方針: **最小修正**。`BuiltScene` の RAII 化は diff とレビュー面積が一桁大きい
(テスト3ファイルの手動 release 改修、move-only 化の全数調査)ため見送り、
known-issues に TODO として残す。

- `src/umbreon/render/embree_renderer.cpp`
  - `device_ = device; scene_ = built.scene;` を `:1739-1740` から
    **`buildEmbreeScene` 成功直後(`:1287` 付近)へ前倒し**。以降の例外
    (allocateFrameBuffers、GI サイドチャネル assign、parallel_for、runPt1GiPass 等)
    はスタック巻き戻しで `~EmbreeRenderer` → `releaseEmbree()`(`:281-292`、
    scene→device の順)が回収する。`EmbreeRenderer` は `pipeline.cpp:113` で
    スタック上構築のため group-alpha 経路も自動カバー。
  - 旧代入2行は削除(所有権移譲点を唯一に保つ)。keep-alive コメントは前倒し先へ併合。
- `src/umbreon/render/scene_build.cpp`(`buildEmbreeScene` `:133-170`)
  - `out.scene = rscene;` 以降の本体(builder 3呼び出し + commit)を try/catch で包み、
    catch で `rtcReleaseScene(rscene); throw;`。既存の commit エラー経路(`:164-168`)は
    「throw のみ」に変えて release を catch に一本化(二重 release を構造的に排除)。
    builder 内の vector 確保からの `bad_alloc` リーク穴も塞がる。

テスト: 例外注入フックがないため新規自動テストなし。既存全テスト green +
refactor_check 全ケース byte-exact + レビューで「移譲点より後に device/scene を
素で参照する経路なし」を確認。

## 4. C2: 問題4 — OIDN の TBB affinity

- `src/umbreon/experimental/irradiance_cache/denoise_oidn.cpp` の `newDevice`(`:64`)と
  `commit()`(`:65`)の間に1行: `device.set("setAffinity", false);`
  OIDN 2.5.0 公式 README がアプリ側も TBB を使う場合の推奨としている。
  PinningObserver 生成自体を抑止し、共有 TBB ワーカーへの affinity 残留を根本遮断。
  `newDevice` はリポジトリでここ1箇所のみ = 全3系統(最終カラー / GI E / pt2 glossy)を
  カバー。
- デバイス常駐化は引き続きスコープ外([oidn-max-memory.md](oidn-max-memory.md) §9 の
  判断を維持)。

テスト: `test_oidn_denoise` green。refactor_check の gi/pt2/pt2em(OIDN E-denoise を
通る)が不変か実測確認。万一差分が出たら C2 を C5 と同じベースライン更新群へ移す。

## 5. C3: 問題3 — giRecordViz/giOcclusion の常時確保(約 300MB + 100MB)

- `src/umbreon/render/render_options.hpp`(公開ヘッダ): `bool giWriteAov = false;` を
  追加(`aoWriteAov` `:62` の前例に倣う)。
- `src/umbreon/render/embree_renderer.cpp`:
  - `allocateFrameBuffers` `:514-515` の `giRecordViz`/`giOcclusion` 確保を
    `if (opt.giWriteAov)` でゲート(`normal`/`position`/`indirect` は実務バッファなので不変)。
  - 書き込みガード: `:927`(pt1/pt2)、`:1233`/`:1240-1247`(cache)を `.empty()` で
    ガード(ホットループ外へ bool を持ち上げ)。計算自体は変えずコピーだけゲート。
- `src/umbreon/render/pipeline.cpp` `:211-220`: ダウンサンプルゲートを per-buffer に分解
  (`boxDownsample` は src を無条件に読むため、一括ゲートのままだと空バッファで OOB/UB):
  `!frame.indirect.empty()` → position/indirect、`!frame.giRecordViz.empty()` →
  giRecordViz、`!frame.giOcclusion.empty()` → giOcclusion。
- `src/umbreon/render/progress_cost_model.hpp` `:262-266`: チャンネル数を
  `(strokeEdges ? 6 : 9) + (giWriteAov ? 4 : 0)` に修正。
- `src/bench/aov_dump.cpp` `:205-215`: `.empty()` ガード + スキップログ。
- bench CLI 配線: `cli.cpp`(`--gi-write-aov`、`--ao-write-aov` の前例に倣う)+
  `scene_setup.cpp:460` 付近。
- **必須テスト改修**: `tests/test_pt1_render.cpp:292-309` が既定オプションで
  `f.giOcclusion[cpix]` を直接読むため `o.giWriteAov = true;` を追加(忘れると UB)。

網羅確認: `giRecordViz|giOcclusion` の全出現10箇所(確保2・書き込み3・ダウンサンプル2・
読み3)を潰し、実装後に同じ grep で再確認。

新規テスト: (1) 既定で両バッファ empty、(2) `giWriteAov=true` でサイズ正、
(3) on/off で `f.color` bitwise 同一、(4) `supersample=2` + gi + giWriteAov=false が
完走(OOB 修正の直接回帰、cache 経路でも1本)。

## 6. C4: 問題5 — エッジパスのキャンセル配線

方針: 内部ヘッダ(edges/*.hpp は install 対象外)のシグネチャ末尾に
`const RenderProgress* progress = nullptr` を追加し、ループ先頭で
`if (progress && progress->cancelRequested())` early return。例外は投げず部分フレーム +
`FrameResult::cancelled` の既存契約を維持。

- stroke 経路: `stroke_edges.hpp/.cpp` → `screen_vector_edges.hpp/.cpp` →
  `stroke_render.hpp/.cpp` に引数追加。挿入点:
  - Stage1 `classifyCracks`: `screen_edge_classify.cpp:569` の parallel_for ラムダ先頭
    (`embree_renderer.cpp:1398` と同形)
  - Stage2 `traceCrackChains`: `screen_edge_trace.cpp:223` の行ループ先頭
  - Stage2.5 `pruneWeakChains`: `screen_edge_prune.cpp:74` の8ラウンド先頭
  - Stage3.5 rewire: `screen_vector_edges.cpp:570` のラウンド先頭
  - Stage4 PASS1/2: `:1113` / `:1509` のチェーンループ先頭
  - `renderStrokeChains`: `stroke_render.cpp:1217` のチェーンループ + `:1394` の
    ラスタ parallel_for
  - `applyScreenVectorEdges` 本体はステージ間で cancelled を見て early return
- obj-edges 経路: `object_space_edges.hpp/.cpp` の `generateObjectSpaceEdges` に引数追加、
  メインループでポーリング。`pipeline.cpp:39` で progress を渡す。
- `pipeline.cpp` 側は progress の受け渡し(`:164-169`)のみ。cancelled フラグ立ては
  直後の既存 Postprocess 境界チェック(`:174-180`)が拾う。
- `tests/test_screen_vector_edges.cpp` はデフォルト引数のため無改修。

新規テスト: `tests/test_render_async.cpp` にケース追加 — `strokeEdges.enable=true` で
`RenderPhase::Edges` を観測したら `requestCancel()`、観測できたときのみ assert
(既存ケース11 `:281-312` のタイミング非依存パターン)。refactor_check の
edges/objedge ケース byte-exact 必須。

## 7. C5: 問題2 — pt2 gather NEE の shadows ゲート(唯一の画像変更)

- `src/umbreon/integrator/pt1/pt1_gather.hpp`(`pt1EvalVertex` `:175-205`):
  `sh` 決定を3分岐に。`if (!p.shadows) sh = 1.0f;` を既存 soft/hard 分岐の前段に追加。
  `stats->neeRays` はシャドウレイを撃ったときのみ加算(`nee_frac` が削減の実測指標になる)。
  pt2_reflect/pt2_glossy も `pt1EvalVertex` 経由なのでこの1箇所で全カバー。
  `p.shadows==true` なら実行列はバイト同一。
- `src/umbreon/render/embree_renderer.cpp`:
  - `:615-616`(runPt1GiPass):
    `gp.shadows = (opt.giIntegrator == 2) ? (opt.shadows || opt.envLights > 0) : true;`
    (合成条件は direct pass の `hit_shader.hpp:155` と同一式。相互参照コメント)
  - `:1183-1184`(runIrradianceCacheGiPass): `gp.shadows = true;` 固定 + コメント
- `src/umbreon/experimental/irradiance_cache/irradiance_cache.hpp`:
  - dead field `shadowSamples`(`:87`)を削除(読み手ゼロ。gather に配線すると RNG
    シードが三角形 ID 由来のため相関バンディングになるので配線しない。pt2 は spp 平均で
    soft を無償実現済み)。詰め側2箇所の代入も削除。
  - `:86` `shadows` のコメントと `:234-240`「always shadow-correct」コメントを更新。
    cache 経路のコード(`:246`)自体は触らない(凍結)。
- pt2 の emissive NEE(pt2_emissive.hpp)はゲートしない(光輸送の正しさに必須)。
  `shadows` の意味は distant/dome ライトの影に限定とコメント明記。

新規テスト(`tests/test_pt2_render.cpp` に追加):

1. pt1 アンカー: 遮蔽シーンで pt1 を shadows=false/true でレンダ → `f.indirect` が
   bitwise 同一(凍結の恒久ガード)
2. pt2 ゲート: 同シーンで pt2 shadows=false/true → `f.indirect` が異なり、false 側の
   平均輝度 ≥ true 側
3. shadows=false の pt2 が run-to-run bit-exact

既存テスト影響なし(pt2 の shadow 系テストは全て `shadows=true` 明示 = ゲート素通しで
バイト同一。相対比較系は両辺同ゲートで成立)。

refactor_check 運用:

1. C4 時点でベースライン生成 → C5 適用後、FAIL が gi/pt2/pt2em の3ケースのみである
   こと、pt1 ほか全ケース OK(= pt1 凍結の担保)を確認
2. 差分方向(pt2 系が明るくなる方向のみ)を目視 + `--pt1-stats` の `nee_frac` で
   削減を実測
3. C5 込みでベースライン再生成

## 8. C6: 問題6 — angularRadius の優先(文書化のみ)

- opt 側ゲート追加は不採用: `lightRadius` 既定 0 のため「シーンが area light を持てば
  pt2 で soft」は文書化済み仕様(POV `area_light`、`test_pt2_render.cpp:290-310`、
  `test_principled.cpp:217-235` が固定)。実害経路(shadow OFF なのに GI シャドウレイ
  残留)は C5 で閉じる。
- `embree_renderer.cpp:447-453` のコメントに優先規則を明文化。`env_dome.hpp:61` /
  `embree_renderer.cpp:1603` の `opt.lightRadius` 直接参照の不整合は known-issues に
  低優先 TODO として残す。

## 9. ドキュメント更新(各コミットに同梱)

- `docs/api/libumbreon.md`: C3 で `giWriteAov` + FrameResult 表に GI AOV 4種を明文化、
  C5 で `shadows` の説明更新、C6 で優先規則
- `docs/quality_presets.md` / `docs/umbreon_cli.md`: `--gi-write-aov`、shadows の GI への効果
- `docs/plans/known-issues-render-perf-and-leaks.md`: 各節冒頭にステータス行
  (修正済み/文書化のみ/残 TODO: BuiltScene RAII 化、OIDN 常駐化、env_dome 不整合)
- `docs/plans/README.md`: 本プラン文書のエントリ追加

## 10. 検証手順(横断)

- 各コミット: `task build` + `task test`(CI 相当は `CTEST_ARGS=-LE needs_lfs task test:static`)
- refactor_check はローカルで必ず実行(gi/pt2 ケースは LFS シーン依存で CI では走らない)。
  C5 以外は全ケース byte-exact が必須
- pt1 bit-identical の担保: 構造的保証(true 強制 + true なら同一実行列)+
  refactor_check pt1 ケース + 新規恒久ガードテスト

## 11. cuemol2 側への連絡事項

1. ABI: `RenderOptions` に `giWriteAov` 追加(C3)→ libumbreon 更新時に cuemol2 の
   再ビルド必須(既定 false なので挙動は無変更)
2. FrameResult: `giRecordViz`/`giOcclusion` は既定で空に。cuemol2 側がこれらを
   読んでいないことの確認を依頼
3. shadows の意味変更(C5): GI ON(pt2)時に shadow トグルが gather にも効く。
   shadow OFF のユーザは出力が変わる(明るく・速く)のでリリースノート必須
4. 速度低下の症状本体(darwinbg 固着)は cuemol2 側で対応
   (`docs/architecture/umbreon-render-qos-throttling.md`。本プランのスコープ外)
