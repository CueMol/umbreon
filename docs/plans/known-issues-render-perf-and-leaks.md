# 判明している問題点: レンダー例外時のリーク / GI と shadow 設定の不整合 / デバッグ用メモリ

CueMol3(tritium)で「GI + soft shadow でレンダリングした後、一旦異常に遅くなると
soft shadow を切っても・GI を切って通常の ray tracing にしても遅いまま。アプリを
再起動すると直る」という報告があり、その原因調査(`main` = `63e7c52` 時点)で
見つかった umbreon 側の問題点をここに書き残す。**プランではない**(対応方針は
別途プラン化する)。

再現条件の絞り込み:
- レンダラ種別(isosurf/cartoon 等)には依存しない。
- GI なしの通常 ray tracing でも発生する(前回の記憶)。
- 遅くなった状態でも**出力画像は正しく、画質も通常のレイトレース品質と変わらない**
  ("同じ計算をただ何倍も遅く実行している"という挙動)。
- 発生中も GUI 操作(回転などの OpenGL 表示)は通常速度。
- 再現性が低く、意図的な再現操作(animation 操作、multi-gradient 操作の再実行)では
  再現しなかった。

**結論: umbreon 側のバグではない。** macOS の task policy 実験(`taskpolicy`
コマンドで renderer プロセスを意図的に darwinbg 化/解除)で、この症状は
**Electron の renderer プロセスが macOS の darwinbg(background task policy)
に落とされたまま解除されない**ことで説明できると判明した(6 倍級の遅化を
再現・維持できるのはこのケースのみ)。renderer プロセス内 = umbreon が動く
プロセスの中からは自己修復できないことも実験で確認済み(スレッド QoS の
明示設定も `setpriority` による自己解除も、外部から掛けられた darwinbg を
打ち消せない)。

原因の特定過程・実験結果・診断コマンド・対策案は cuemol2 リポジトリ側
(呼び出し元である tritium/Electron の環境要因のため)に書いた:
`docs/architecture/umbreon-render-qos-throttling.md`。

以下は、その調査の過程で見つかった、**この症状とは独立に実在する** umbreon の
バグ・問題点。

**ステータス(2026-08-11)**: 6件すべて branch `fix/known-issues-render-perf-and-leaks`
で対応済み(修正プランは [fix-known-issues-render-perf-and-leaks.md](
fix-known-issues-render-perf-and-leaks.md))。各節冒頭に個別ステータスを付記。
残 TODO は以下の3点(いずれも低優先):

- `BuiltScene` の RAII 化(現状は所有権移譲の前倒し + try/catch で実害を閉じた。
  型としての安全化は将来の refactoring)
- OIDN デバイスの常駐化(oidn-max-memory.md §9 から引き続き先送り。affinity 問題は
  `setAffinity=false` で対処済みなので、残る利得はデバイス生成コストのみ)
- `env_dome.hpp` / adaptive-AA boost が `opt.lightRadius` を直接読む不整合(問題6参照)

---

## 1. 例外発生時に RTCDevice + BVH(RTCScene)がリークする(高)

**修正済み**: `device_`/`scene_` への所有権移譲を `buildEmbreeScene()` 成功直後へ
前倒しし(以降の例外は `~EmbreeRenderer` → `releaseEmbree()` が回収)、
`buildEmbreeScene()` 内部も try/catch で部分 scene の解放を一本化した。

`src/umbreon/render/embree_renderer.cpp`。

`buildEmbreeScene()` の呼び出しだけは try/catch で `device` の解放が保証されている
(`:1279-1286`):

```cpp
  BuiltScene built;
  try {
    built = buildEmbreeScene(device, scene, opt.strokeEdges.enable);
  } catch (...) {
    if (device != sharedDevice_) rtcReleaseDevice(device);
    throw;
  }
```

しかし `device_`/`scene_` への所有権移譲(= `~EmbreeRenderer` が解放できるようになる
唯一のタイミング)は関数の一番最後(`:1736-1740`)まで行われない:

```cpp
  // Keep the device + committed scene ALIVE so the edge pass ... can ray-cast
  device_ = device;
  scene_ = built.scene;
```

つまり **`buildEmbreeScene()` 成功後から `:1736` に到達するまでの間に例外が飛ぶと、
`RTCDevice` と `RTCScene`(BVH 実体)の両方が永久にリーク**する。この区間には
現実的に例外が飛びうる箇所が複数ある:

- `allocateFrameBuffers()`(`:463-519`)の `std::vector::assign` — 高解像度 ×
  supersample で数百 MB 単位の確保
- GI サイドチャネルの `assign`(`:1356-1371`) — `giGroup`/`giGeom`/`giRefl`/
  `giElig`/`reflAmt`/`reflAlpha`/`reflF0`/`reflTan`/`reflAniso`
- `tbb::parallel_for` 本体(`:1393`, `:1515`, `:1619`)からの再送出
- `runPt1GiPass` 内の各種確保

`BuiltScene`(`render/scene_build.hpp:62-64`)にはデストラクタがなく
"caller owns `scene` and must `rtcReleaseScene()` it" とコメントで書かれている
だけなので、このパスでは誰も解放しない。

リークする実体は分子メッシュ全体の BVH(`RTC_BUILD_QUALITY_HIGH` = spatial split
付きでビルドが重く、メモリも標準の MEDIUM より大きい。`scene_build.cpp:144`)+
Embree バッファへコピーされた de-index 済み頂点(position/normal/color、頂点あたり
40B、`buildTriangleMesh`, `scene_build.cpp:44-74`)なので、1 回のリークで
数百 MB に達しうる。`std::bad_alloc` がトリガーになりやすい経路のため、
「メモリ逼迫時にだけ、低い頻度で起きる」という性質を持つ。

group-alpha(半透明セクションが1つでもある)経路では `device` は RAII の
`SharedDevice`(`umbreon.cpp:23`)で守られているが、**`built.scene`(BVH)は
同じ理由でリークする**。

## 2. GI ON のとき `shadows` / `shadowSamples` が GI gather に一切効かない(中〜高)

**修正済み(pt2 限定)**: `pt1EvalVertex` が `p.shadows` を尊重するようになり、
pt2(既定)の詰め側だけが direct pass と同じ合成条件
(`opt.shadows || opt.envLights > 0`)を配線する。pt1/cache は凍結契約のため
常時 shadow-correct を維持(詰め側で true を強制)。dead field の
`shadowSamples` は削除(gather に配線すると RNG シードが三角形 ID 由来のため
相関バンディングになる。pt2 の soft NEE は spp 平均で実現済み)。
shadows OFF の pt2 出力は変わる(遮蔽部の間接光が明るく・gather のシャドウレイが
消えて速くなる: 1ab0 ゲートシーンで nee_frac 0.10 → 0.00)。

`RenderOptions` から詰められてはいる(`embree_renderer.cpp:615-616`,
`:1184`)が、受け側のフィールドは pt1/pt2 のどこからも読まれていない:

`experimental/irradiance_cache/irradiance_cache.hpp:86-87`
```cpp
  bool shadows = false;   // include direct shadows in the gather
  int  shadowSamples = 1;
```

GI ゲザー頂点の NEE は常に無条件実行される:

`experimental/irradiance_cache/irradiance_cache.hpp:236-246`
```cpp
  // ... (This is decoupled from the primary render's `shadows` flag: the GI
  // gather is always physically shadow-correct even when the direct pass draws
  // no hard shadows.) ...
    const float sh = computeShadow(p.scene, Py, Ng, Ny, eps, l, 1, s0, s1);
```
`integrator/pt1/pt1_gather.hpp:196` も同様。

**帰結**: GI ON の状態で UI 側の shadow/soft-shadow を OFF にしても、GI のコスト
(レンダー時間の 71〜90%、`embree_renderer.cpp:1696-1697` の実測コメントより)は
ほとんど減らない。減るのはダイレクトパスのシャドウレイのみ(`shading/shading.hpp:
107-110`)。「soft shadow を切っても遅いまま」という今回の観察の一部は、
バグではなくこの仕様どおりの挙動である可能性が高い(GI 自体を OFF にしても遅かった
かどうかは未確認)。

意図的な仕様(コメントに明記)なのか、単に実装漏れなのかは要検討。少なくとも
UI 側が「shadow 設定は GI に効かない」ことを知らずに露出しているなら UX 上の
不整合。

## 3. GI ON 時、デバッグ用 AOV `giRecordViz` が常時確保される(中、メモリ)

**修正済み**: `RenderOptions::giWriteAov`(既定 false)を新設し、`giRecordViz` と
`giOcclusion` の確保・書き込み・ダウンサンプル・dump を全てゲートした
(既定で約 npix*4 float、1920x1440×ss3 なら約 400MB の節約)。bench には
`--gi-write-aov` を追加。

`embree_renderer.cpp:514`(確保)、`pipeline.cpp:216-217`(ダウンサンプル)。
`opt.gi == true` であれば常にヒートマップ用バッファ(`npix*3` float)を確保・
処理する。可視化専用で通常のレンダリングには不要なはずだが、ゲートするフラグが
存在しない。1920x1440 出力 × supersample 3 なら単体で約 300MB。

## 4. OIDN がデノイズ呼び出しごとに TBB ワーカーへ affinity を打つ(中、環境依存)

**対処済み**: デバイス commit 前に `device.set("setAffinity", false)` を設定し、
`PinningObserver` の生成自体を抑止した(OIDN マニュアルが「アプリ側も TBB を使う
場合」の推奨としている構成)。出力は bit-exact(refactor_check の OIDN 経路
ケース含む)。デバイス常駐化は引き続きスコープ外。

`experimental/irradiance_cache/denoise_oidn.cpp:64-65` で毎回
`oidn::newDevice(oidn::DeviceType::CPU)` を作り、関数末尾で破棄する。deplibs の
OIDN 2.5.0 は内部で `PinningObserver`(`tbb::task_arena` + `task_scheduler_observer`
経由でプロセス共有の TBB ワーカーにスレッドアフィニティを設定)を使う。

1 レンダーあたり最大 3 系統(最終カラー denoise、pt1 の GI E バッファ denoise、
pt2 glossy denoise)、group-alpha 多パスならさらに `1 + グループ数` 倍呼ばれる。
observer の破棄前にアフィニティが正しく復元されない経路があると、共有ワーカーに
affinity が残留し、以後の TBB 処理全体が遅くなりうる。

**Apple Silicon では `thread_policy_set` 系がおそらく効かない(KERN_NOT_SUPPORTED)
ため実害は薄いと推測されるが、Intel Mac / Linux ビルドでは検証していない。**
「一度遅くなると戻らない」症状の TBB affinity 説は QoS 説と混同しやすいので、
切り分けが必要(`denoiser=0` かつ `giDenoise=false` で OIDN を一切通さずに
再現するかどうかで判別できる)。

## 5. エッジパスにキャンセルチェックポイントが一つもない(低〜中)

**修正済み**: `const RenderProgress* progress = nullptr` を edges/ の各関数に配線し、
classify の行チャンク・trace のコーナ行/ループ・prune の8ラウンド・junction rewire の
ラウンド・Stage4 の両チェーンパス・stroke 描画のチェーンループとラスタ行チャンク・
obj-edges の emit ループでポーリングするようにした。キャンセル時は部分フレーム +
`FrameResult::cancelled`(既存契約どおり)。null なら従来どおりゼロオーバーヘッド。

キャンセルチェックの全箇所(`grep -rn cancelRequested src/umbreon/`):
`embree_renderer.cpp:1398, 1515, 1621, 1707`、`pt1_gather.hpp:553`、
`denoise_oidn.cpp:26`、`pipeline.cpp:138, 176`(いずれもフェーズ境界のみ)。

**エッジパス全体(`src/umbreon/edges/`)にキャンセルチェックが存在しない**
(`grep -rn cancel src/umbreon/edges/` はコメント1件のみ)。
`RenderTask::Impl::~Impl`(`umbreon.cpp:282-287`)は `requestCancel()` してから
`join()` するため、キャンセル直後に次のレンダーを開始すると、旧レンダーの
エッジパス(ほぼシングルスレッド)が完走するまで両者が CPU を奪い合う。

## 6. pt2 では `angularRadius`(シーン側)が `lightRadius`(オプション側)を上書きしうる(低、現状未実害)

**文書化のみで対応**: 問題2の修正により、`shadows` OFF なら direct にも pt2 gather
NEE にもシャドウレイ自体が飛ばなくなったため、「UI で影を切ったのにシーン側の値で
シャドウ処理が残る」経路は閉じた(radius は撃たれるレイの penumbra 形状にしか
効かない)。優先規則は `buildSceneLights` のコメントと docs/api/libumbreon.md に
明文化。`env_dome.hpp:61` と adaptive-AA boost(`embree_renderer.cpp:1603`)が
`opt.lightRadius` を直接読む不整合は残 TODO(低優先)。

`embree_renderer.cpp:451-453`:
```cpp
    l.radius = (opt.giIntegrator == 2 && dl.angularRadius > 0.0f)
                   ? dl.angularRadius
                   : radians(opt.lightRadius);
```
`giIntegrator == 2`(cuemol2 の GI ON 既定)のとき、`Scene::DistantLight::
angularRadius`(`scene.hpp:398-403`)が設定されていれば `opt.lightRadius` より
優先される。現状 cuemol2 は `angularRadius` を一切設定していない
(`UmbreonDisplayContext.cpp` はライトの direction/color/intensity/
castsHighlight のみ設定)ので今すぐの実害はないが、**「UI で soft shadow を
OFF にしたのに、シーン側の値のせいで内部的には有効のまま残る」唯一の経路**
として存在する。ao_dome 側 (`ao/env_dome.hpp:61`) も同様に `opt.lightRadius`
を直接使う独立経路がある。

---

## 参考: cuemol2 側で既に修正した関連バグ

同じ調査で見つかった cuemol2 側(呼び出し元)の問題は cuemol2 リポジトリの
PR #475 で修正済み:
- `UmbreonBackend.beginInProcess` で `beginRender()` が例外を投げると scene が
  detach されないまま残る(umbreon 側ではなく呼び出し側の bug)。
- `UmbreonDisplayContext::drainLog()` を呼ばない経路(同期 `write()`)で
  プロセスグローバルなログバッファが無制限に伸びる — 1MiB 上限を追加。
- `Scene::display()`/`processHit()` の `StyleMgr` context push が非 RAII で、
  レンダラが例外を投げるとコンテキストスタックが壊れたまま残る。

これらは umbreon 本体のバグではないので上記の番号付きリストには含めていない。
