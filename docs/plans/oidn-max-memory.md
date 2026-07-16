# OIDN in-process + maxMemoryMB(内蔵タイル分割 denoise)実装プラン

対象ブランチ: `feat/oidn-max-memory`(main = 6b104d1 から分岐)。
このプランは実装者(別セッションの AI/人間)が**このファイルだけで**作業できるよう自己完結で書いてある。

---

## 1. 背景(なぜこの変更か)

- CueMol3(Electron)で GI デノイズ(OIDN)を使うと、renderer プロセスの
  **PartitionAlloc が SIGTRAP でクラッシュ**する(~1200x1200 supersample 3 で発生。
  cuemol2 リポジトリ `docs/architecture/umbreon-process-isolation.md` 参照)。
  現在 cuemol2 側は `OIDN_MAX_RENDER_AREA`(6MP)キャップで回避している。
- 一度は OIDN を別プロセス worker に分離する実装を行った(**PR #59、ブランチ
  `feat/oidn-out-of-process`。現在 draft で保留中。マージ・rebase してはならない。
  参照専用**)。しかし worker 方式は配備が煩雑(exe 配置、Electron の
  qif→TS→packaging のパス配線、macOS 個別署名)なため不採用とし、
  **in-process のまま OIDN の `maxMemoryMB` パラメータ(内蔵タイル分割)で
  クラッシュを回避する方式**に切り替えることになった。
- main は worker 化前の状態(OIDN は libumbreon に静的リンク、in-process 実行)
  なので、この変更は **main への小さな追加**であり、revert 作業は不要。

## 2. 実測データ(この設計の根拠。2026-07 に macOS arm64 + deplibs OIDN 2.5.0 で計測済み)

| 条件 | 最大**単一** malloc 確保 | 13MP denoise 時間 |
|---|---|---|
| 3600x3600 (13MP), maxMemoryMB 未設定(既定) | **2524.9 MiB** | 10.1s |
| 3600x3600, maxMemoryMB=1024 | 889.2 MiB | 12.4s (+23%) |
| 3600x3600, maxMemoryMB=512 | 528.6 MiB | 19.1s (+89%) |
| 7680x4320 (8K, 33MP), maxMemoryMB=1024 | **953.5 MiB** | - |

わかったこと:
1. 既定設定では OIDN はスクラッチを**単一の ~2.5GiB malloc** で確保する(13MP時)。
   PartitionAlloc の単一確保上限(~2GiB)を超える。これがクラッシュの直接原因。
   (OIDN 既定の「>2K 解像度は総量 ~6GB 制限」は総量の話で、単一アリーナは救わない。)
2. `maxMemoryMB=1024` で最大単一確保は**解像度非依存**に ~0.9-1.0GiB に収まる
   (8K でも 953MiB)。2GiB 上限に対し2倍のマージン。
3. 仕様上 maxMemoryMB は「要求であって保証ではない」(512 指定でも 528MiB と微超過)
   が、1024 指定のマージンなら実用上問題ない。
4. 性能コストは 1024 で +23%。512 の +89% は割に合わない。**既定値は 1024 とする。**

計測方法(再検証したい場合): scratchpad に malloc interposer
(`__DATA,__interpose` で malloc/calloc/realloc/valloc/aligned_alloc/posix_memalign
をフックし最大サイズを記録する dylib)+ OIDN を直接呼ぶ probe
(画像バッファは mmap で確保して malloc 統計を汚さない)を作り、
`DYLD_INSERT_LIBRARIES` で実行する。注意: `/usr/bin/time` 等の SIP 保護バイナリ経由
では DYLD_* が剥がされるので直接実行すること。

## 3. 設計決定(確定事項。再検討不要)

- `RenderOptions::oidnMaxMemoryMB`(int)を追加。**既定 1024**。`< 0` = OIDN 既定(cap なし)。
- OIDN filter 設定に `filter.set("maxMemoryMB", opt.oidnMaxMemoryMB)` を追加(`>= 0` のときのみ)。
- **フォールバック報告**を追加(worker 版 PR #59 から概念を移植):
  - `FrameResult::denoiserUsed` / `FrameResult::pt1DenoiserUsed`(int; 0=None,
    1=AtrousBilateral, 2=OIDN。「実際に走ったもの」)。
  - `denoiseOidn` を `bool` 返却に変更(true = OIDN が実行された)。
  - `denoisePt1E` を `int` 返却に変更(走った backend、0=no-op)。
  - **意図的な挙動変更**: 旧実装は OIDN の device/execute エラー時に警告だけ出して
    デノイズをスキップしていた。新実装は呼び出し側で `denoiseAtrous` にフォールバック
    する(値 1 を報告)。エラー経路のみの変更で、正常系の画素出力は不変。
- 公開 API 関数(probe 等)は**追加しない**(in-process では可否がコンパイル時に決まり、
  実行時検知は denoiserUsed で足りる)。
- 決定性: maxMemoryMB の値が変わるとタイル境界が変わり、タイリングが発動する解像度では
  **出力ビットが変わる**。値は固定既定にする。小さい画像(refactor_check の 256x256 等)
  ではタイリング不発動なので既定 1024 でもビット不変のはず(§6 で検証)。

## 4. main の現状コード地図(アンカー。行番号は目安、ずれたら文字列で grep)

| ファイル | 位置 | 内容 |
|---|---|---|
| `src/umbreon/experimental/irradiance_cache/denoise_oidn.cpp` | 全体(~95行) | in-process OIDN 実装。`denoiseOidn(FrameResult&, const RenderOptions&)`。`filter.set("cleanAux", ...)` が61行、`filter.commit()` が62行付近 |
| `src/umbreon/experimental/irradiance_cache/denoise.hpp` | 35行付近 | `denoiseOidn` 宣言(void)。内部ヘッダ(非インストール) |
| `src/umbreon/render/pipeline.cpp` | 185-196行付近 | 最終カラー denoiser ディスパッチ(`#ifdef UMBREON_HAVE_OIDN` で denoiseOidn / #else 警告+atrous / else-if atrous) |
| `src/umbreon/experimental/pt1/pt1_denoise.cpp` | 全体(~70行) | `detail::denoisePt1E`(void)。E バッファを合成 FrameResult に詰めて denoiseOidn(#ifdef)/atrous(#else) |
| `src/umbreon/experimental/pt1/pt1_integrator.hpp` | 62行 | `denoisePt1E` 宣言 |
| `src/umbreon/render/embree_renderer.cpp` | 564行 / 654行 | `denoisePt1E` の2呼び出し点(if/else 排他)。どちらも直後で `res.pt1Timing.denoise` に代入しており `res`(FrameResult)が到達可能 |
| `src/umbreon/render/frame_result.hpp` | 末尾 `Pt1RayCounts pt1Rays;` の後 | フィールド追加位置 |
| `src/umbreon/render/render_options.hpp` | 181行 `oidnCleanAux` | フィールド追加位置(公開ヘッダ) |
| `src/bench/cli.hpp` | 84行 / `src/bench/cli.cpp` 501-505行(`--oidn-clean-aux`) / `src/bench/scene_setup.cpp` 440行 | CLI 配管の追加位置(既存 oidn フラグの隣) |
| `CMakeLists.txt` | 159-180行 | OIDN ブロック(`UMBREON_WITH_OIDN` → `find_package(OpenImageDenoise)` → `UMBREON_HAVE_OIDN` PRIVATE 定義)。この後 185行で `add_subdirectory(tests)` なので `OpenImageDenoise_FOUND` は tests/ から見える |
| `tests/CMakeLists.txt` | `if(TARGET umbreon)` ブロック | レンダ系テスト登録場所。ハーネスは `tests/test_util.hpp` の `umbreon::test::Suite`(各テスト = 独立 main) |

重要な既存規約: `UMBREON_HAVE_OIDN` は **target-private マクロ**。評価は libumbreon の
.cpp TU 内のみで行い、ヘッダに inline 定義を置かない(ODR。`pt1_denoise.cpp` 冒頭の
コメント参照)。

## 5. 変更内容(ファイル別)

### 5.1 `src/umbreon/render/render_options.hpp`(公開ヘッダ)

`oidnCleanAux` の直後に追加:

```cpp
  // OIDN scratch memory cap in MB (< 0 = OIDN default, no cap). OIDN denoises
  // in internal overlapping tiles to stay under the cap, which bounds every
  // SINGLE allocation regardless of resolution. Required inside Electron:
  // PartitionAlloc aborts (SIGTRAP) on single allocations > ~2 GiB, and the
  // uncapped default allocates one ~2.5 GiB arena at 13 MP. Measured with
  // 1024: max single allocation ~0.9-1.0 GiB up to 8K, ~+23% denoise time at
  // 13 MP. NOTE: the value changes the tile layout and thus the output bits
  // at resolutions large enough to trigger tiling -- keep it fixed for
  // reproducibility (small frames are tiling-free and bit-identical).
  int oidnMaxMemoryMB = 1024;
```

### 5.2 `src/umbreon/experimental/irradiance_cache/denoise_oidn.cpp`

- シグネチャを `bool denoiseOidn(...)` に変更。
- 冒頭の degenerate ガード(`W <= 0 || ...`)は `return false;`。
- device init 失敗の早期 return は `return false;`、execute 後のエラーチェックも
  `return false;`。正常完了で `return true;`(書き戻しループは現状のまま)。
- `filter.commit()` の直前に追加:

```cpp
  // Cap the scratch arena; uncapped, OIDN allocates a single multi-GiB block
  // at high resolutions (measured 2.5 GiB at 13 MP), which aborts hosts whose
  // allocator rejects huge single allocations (Electron PartitionAlloc,
  // ~2 GiB). OIDN implements the cap as internal overlapping tiling.
  if (opt.oidnMaxMemoryMB >= 0) filter.set("maxMemoryMB", opt.oidnMaxMemoryMB);
```

- ファイル冒頭コメントに「fallback は呼び出し側(pipeline.cpp / pt1_denoise.cpp)が行う。
  返り値は FrameResult::denoiserUsed 用」の旨を追記。

### 5.3 `src/umbreon/experimental/irradiance_cache/denoise.hpp`

`denoiseOidn` 宣言を `bool` に変更し、doc コメントへ追記:
「returns true when OIDN processed the frame; false on any failure (callers
fall back to denoiseAtrous and report DenoiserBackend::AtrousBilateral)」。

### 5.4 `src/umbreon/render/pipeline.cpp`(185-196行のディスパッチ)

```cpp
  if (opt.denoiser == static_cast<int>(DenoiserBackend::OIDN)) {
#ifdef UMBREON_HAVE_OIDN
    if (denoiseOidn(frame, opt)) {
      frame.denoiserUsed = static_cast<int>(DenoiserBackend::OIDN);
    } else {
      // OIDN failed at runtime (device/filter error): fall back instead of
      // silently skipping the denoise (behavior change from the old code,
      // which left the frame un-denoised on this rare path).
      denoiseAtrous(frame, opt);
      frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
    }
#else
    std::fprintf(stderr, ...既存の警告文そのまま...);
    denoiseAtrous(frame, opt);
    frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
#endif
  } else if (opt.denoiser != static_cast<int>(DenoiserBackend::None)) {
    denoiseAtrous(frame, opt);
    frame.denoiserUsed = static_cast<int>(DenoiserBackend::AtrousBilateral);
  }
```

`denoiser == None` はフィールド初期値 0 のまま(既定経路は一切触らない)。

### 5.5 `src/umbreon/render/frame_result.hpp`

`Pt1RayCounts pt1Rays;` の後に追加:

```cpp
  // Which denoiser actually ran (DenoiserBackend as int, matching
  // RenderOptions::denoiser: 0=None, 1=AtrousBilateral, 2=OIDN). denoiserUsed
  // is the final-color denoise stage: 2 only when OIDN really processed the
  // frame; 1 covers both an explicit a-trous request AND every OIDN fallback
  // (backend not built, or an OIDN runtime error). pt1DenoiserUsed reports
  // the pt1 integrator's internal indirect-irradiance (E) denoise
  // (RenderOptions::pt1Denoise) the same way; 0 when that stage did not run.
  // This is the field CueMol's GI path reads (giDenoise maps to pt1Denoise).
  // Group-alpha multipass renders report the final (carrier) pass. Additive
  // fields: pixel output is unaffected.
  int denoiserUsed = 0;
  int pt1DenoiserUsed = 0;
```

### 5.6 `src/umbreon/experimental/pt1/pt1_integrator.hpp` + `pt1_denoise.cpp`

- 宣言を `int denoisePt1E(...)` に変更、doc コメントに「returns the
  DenoiserBackend that ran (2=OIDN, 1=a-trous fallback, 0=degenerate no-op)」。
- `pt1_denoise.cpp`: degenerate ガードは `return 0;`。本体:

```cpp
  int used;
#ifdef UMBREON_HAVE_OIDN
  if (denoiseOidn(tmp, dopt)) {
    used = static_cast<int>(DenoiserBackend::OIDN);
  } else {
    denoiseAtrous(tmp, dopt);  // rare runtime-error fallback
    used = static_cast<int>(DenoiserBackend::AtrousBilateral);
  }
#else
  std::fprintf(stderr, ...既存の警告文そのまま...);
  denoiseAtrous(tmp, dopt);
  used = static_cast<int>(DenoiserBackend::AtrousBilateral);
#endif
  ...既存の write-back ループ...
  return used;
```

### 5.7 `src/umbreon/render/embree_renderer.cpp`(2箇所、564/654行付近)

`detail::denoisePt1E(...)` 呼び出しを `res.pt1DenoiserUsed = detail::denoisePt1E(...);`
に変更(timing 計測の構造はそのまま)。2経路は if/else 排他なので上書き競合なし。

### 5.8 CLI 配管(bench 側)

- `src/bench/cli.hpp`: `oidnCleanAux` の隣に `int oidnMaxMemoryMB = 1024;`。
- `src/bench/cli.cpp`: `--oidn-clean-aux` パースの直後に
  `--oidn-max-memory <MB>`(`std::atoi`)を追加。usage 文字列にも1行追加
  (`[1024]`、`-1 = OIDN default` の旨)。
- `src/bench/scene_setup.cpp`: `ropt.oidnCleanAux = ...` の直後に
  `ropt.oidnMaxMemoryMB = opt.oidnMaxMemoryMB;`。

### 5.9 ドキュメント `docs/api/libumbreon.md`(最小限)

- §4.7 FrameResult 表に `denoiserUsed` / `pt1DenoiserUsed` の2行を追加
  (0/1/2 の意味、fallback は 1、CueMol の GI 経路が見るのは pt1DenoiserUsed)。
- §6「パフォーマンスとエラー処理」に bullet を1つ追加: OIDN は
  `oidnMaxMemoryMB`(既定 1024)で内蔵タイル分割され、単一確保が解像度非依存に
  ~1GiB 以下に収まる(Electron PartitionAlloc の ~2GiB 単一確保上限対策。
  13MP 実測: 既定 cap なしだと単一 2.5GiB)。
- §7「CueMol 統合の指針」に1行: これにより cuemol2 の `OIDN_MAX_RENDER_AREA`
  キャップ(UmbreonDisplayContext.cpp)は撤去できる(Electron 実機での確認後)。

## 6. テスト(既存 Suite 流儀。最小限、新規バイナリは1本のみ)

### 6.1 tests/CMakeLists.txt のゲート

ルート CMakeLists の OIDN ブロックは `add_subdirectory(tests)` より前なので、
tests/ から `UMBREON_WITH_OIDN` と `OpenImageDenoise_FOUND` が見える。

```cmake
  # (if(TARGET umbreon) ブロック内)
  if(UMBREON_WITH_OIDN AND OpenImageDenoise_FOUND)
    # OIDN 有無で期待値が変わるテスト用
    target_compile_definitions(test_pt1_render PRIVATE UMBREON_TEST_HAVE_OIDN)
    add_executable(test_oidn_denoise test_oidn_denoise.cpp)
    target_link_libraries(test_oidn_denoise PRIVATE umbreon)
    add_test(NAME oidn_denoise COMMAND test_oidn_denoise)
  endif()
```

### 6.2 `tests/test_pt1_render.cpp` に追記(既存チェックの後)

```cpp
  // pt1DenoiserUsed reporting. pt1Denoise off => stage never runs (0). On =>
  // OIDN (2) when the backend is built, a-trous fallback (1) otherwise.
  {
    umbreon::RenderOptions o = makeGiOptions(1, 1.0f);
    o.pt1Denoise = false;
    s.check("pt1DenoiserUsed == 0 when pt1Denoise off",
            umbreon::render(sc, o).pt1DenoiserUsed == 0);
  }
  {
    umbreon::RenderOptions o = makeGiOptions(1, 1.0f);
    o.pt1Denoise = true;
#ifdef UMBREON_TEST_HAVE_OIDN
    s.check("pt1DenoiserUsed == 2 (OIDN built)",
            umbreon::render(sc, o).pt1DenoiserUsed == 2);
#else
    s.check("pt1DenoiserUsed == 1 (a-trous fallback, OIDN not built)",
            umbreon::render(sc, o).pt1DenoiserUsed == 1);
#endif
  }
```

### 6.3 `tests/test_render_shadows_gi.cpp` に追記(GI 決定論ブロック内、`a` を流用)

```cpp
    // RenderOptions defaults denoiser to None => reports 0; explicit a-trous
    // reports 1. (The GI-conditional a-trous default lives in the bench CLI,
    // not the library.)
    s.check("denoiserUsed == 0 when denoiser None (default)", a.denoiserUsed == 0);
    umbreon::RenderOptions atr = o;
    atr.denoiser = 1;
    s.check("denoiserUsed == 1 for an explicit a-trous render",
            umbreon::render(sc, atr).denoiserUsed == 1);
```

### 6.4 新規 `tests/test_oidn_denoise.cpp`(OIDN ビルド時のみ登録)

合成ノイズフレームのビルダーと分散ヘルパは PR #59 ブランチのテストから流用できる
(読み取り専用で `git show origin/feat/oidn-out-of-process:tests/test_oidn_client_fallback.cpp`
の `makeNoisyFrame`、`git show origin/feat/oidn-out-of-process:tests/test_oidn_worker_ipc.cpp`
の `lumStats`/`allFinite`/`bgUntouched` を参照。ブランチ自体は触らない)。

チェック内容(64x48、フラット2領域+決定的ノイズ+背景行、albedo/normal ガイド付き):
1. `denoiseOidn(f, opt) == true`(OIDN が実行された)
2. フラット領域の分散が減少(`< 0.5 *` 前)
3. 背景(zero normal)ピクセルはバイト不変
4. 出力が全て有限
5. `denoiseAtrous` を同条件で当てた結果と**不一致**(OIDN が本当に走った判別)
6. **maxMemoryMB 不変性**: 同一入力を `oidnMaxMemoryMB = 1024` と `-1` で denoise し
   **ビット一致**(小画像ではタイリング不発動 = 既定値 1024 が小画像の出力を変えない
   ことの固定。§6.6 の refactor_check 前提を単体テストでも守る)
7. render() レベル: 小 GI シーン(test_pt1_render の makeGiScene 相当を簡易に内製)を
   `denoiser = 2` でレンダし `denoiserUsed == 2`

### 6.5 実行

`task test`(= configure:static + build + ctest)。deplibs 環境では OIDN 有効なので
新テストが実行される。

### 6.6 byte-exact ゲート

`./scripts/refactor_check.sh check` が **8/8** であること(特に pt1 ケース:
256x256 の pt1 denoise が OIDN 経路を通る。既定 `oidnMaxMemoryMB=1024` でも小画像は
タイリング不発動なのでビット不変のはず)。
**万一 pt1 ケースが不一致になった場合**: 既定値を `-1`(OIDN 既定 = 旧挙動とビット同一)
に変更して再検証し、「CueMol 側で 1024 を明示設定する」方針に切り替える(この場合
docs の記述もその旨に直す)。ベースラインの再生成はしない。

## 7. 検証手順(実装完了の受入条件)

1. `task test` 全通過(OIDN 有効構成)。
2. `./scripts/refactor_check.sh check` → 8/8。
3. 13MP スモーク(クラッシュ報告サイズ相当):
   ```
   build/umbreon_cli data/1ab0_scene1.pov -o /tmp/oidn13mp.png \
     -W 3600 -H 3600 --threads 4 --gi on --integrator pt1 --quality draft \
     --seed 1 --pt1-stats on
   ```
   正常終了し、stderr に `oidn: device ... filter ... execute ... (3600x3600)` 行が
   出ること(= 13MP を cap 付き OIDN が実処理)。
4. bare 構成(`cmake -S . -B <dir>` を OIDN/Boost prefix なしで configure)でも
   ビルドが通ること(#else 経路のコンパイル確認)。
5. (任意)§2 の alloc probe を再実行し、13MP で最大単一確保 < 1GiB を確認。

## 8. 実装上の注意(リポジトリ規約含む)

- コードコメントは**英語・ASCII のみ**。コミットメッセージは英語、
  `Co-Authored-By` 行は入れない。
- テストは最小限(CLAUDE.md 方針)。上記のセット以上に増やさない。
- `UMBREON_HAVE_OIDN` は target-private。**ヘッダに inline 定義を置かない**
  (denoiseOidn/denoisePt1E の定義は .cpp のみ。既存規約)。
- `DenoiserBackend` enum は内部ヘッダ(`denoise.hpp`)所属。公開ヘッダ側
  (frame_result.hpp / render_options.hpp)は int + コメントで値を列挙する既存流儀。
- ビルド/テストは Taskfile 経由(`task test`)。configure キャッシュが system 構成で
  汚れている場合があるので、挙動が変なら `build/` を消して `task test`。
- `feat/oidn-out-of-process` ブランチと PR #59 は**保留中の参照物**。
  マージ・rebase・push しない。テストコードの流用は `git show` で読むだけにする。
- 画素出力に影響してよいのは「OIDN ランタイムエラー時に atrous フォールバック」の
  1点のみ(§3)。それ以外の変更で refactor_check が崩れたらバグなので原因を直す。

## 9. スコープ外(この PR ではやらない)

- cuemol2 側の変更: `OIDN_MAX_RENDER_AREA` キャップ撤去、Electron 実機での
  1200x1200 ss3 確認(このプランの成果がマージされた後、cuemol2 の別 PR で実施)。
- worker 方式(PR #59)の close/復活の判断。
- OIDN device の常駐化(現状どおり denoise 呼び出しごとに生成・破棄)。
