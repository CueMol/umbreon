# CueMol への out-of-process OIDN worker 配備ガイド

libumbreon の OIDN デノイザは **別プロセス** `umbreon_oidn_worker` で動く。このガイドは、
libumbreon を静的リンクする CueMol（レガシー uxp_gui / CueMol3 tritium）で worker を
**どこにインストールし、どう場所を特定し、どう実行させるか**をファイルレベルで示す。

> 対象読者は CueMol 側の実装者。umbreon 側の該当実装は済んでおり（`umbreon::umbreon_oidn_worker`
> の export、`RenderOptions::oidnWorkerPath`、公開 API `oidnDenoiserAvailable()` /
> `shutdownOidnDenoiser()`、`FrameResult::denoiserUsed` / `pt1DenoiserUsed`）、本ガイドは
> それを消費する手順書である。

---

## 1. 概要

```
libcuemol2 (render module)
   └─ libumbreon (IPC client; Boost.Process/Interprocess は header-only)
        └─(spawn / pipe / file-backed shared memory)→ umbreon_oidn_worker
                                                        （OIDN をリンクする唯一のバイナリ）
```

- **これで消える問題**: OIDN の UNet アロケーションが worker プロセス側に移るため、Electron
  renderer の PartitionAlloc ~2GiB SIGTRAP（`cuemol2/docs/architecture/umbreon-process-isolation.md`）は
  発生原因ごと消滅する。従来の `OIDN_MAX_RENDER_AREA` キャップは撤去できる（§7）。
- **共有メモリ**: umbreon は POSIX shm ではなく**メモリマップファイル**（`file_mapping`）を使う。
  上限はディスク空き容量とアドレス空間のみで、4K（color+albedo+normal ≈ 285MB）を実測で処理済み。
  cuemol2 メモが懸念した「POSIX shm 数十 MB 上限」は該当しない。
- **フォールバック**: worker 不在／死亡／エラー時は内蔵 a-trous に自動フォールバック（stderr 警告）。
  実績は `FrameResult::pt1DenoiserUsed`（GI 経路が使う）/ `denoiserUsed` で検知できる。

## 2. umbreon 側が提供するもの（前提）

- exported target **`umbreon::umbreon_oidn_worker`**（IMPORTED executable）。`UMBREON_WITH_OIDN`
  ビルドのみ存在。package component ではないので `if(TARGET umbreon::umbreon_oidn_worker)` で判定。
- `RenderOptions::oidnWorkerPath`（`std::string`）。非空なら**唯一の候補**。
- 探索順（空のとき）: env `UMBREON_OIDN_WORKER` → ホスト実行ファイル隣接 → PATH。
- 公開 API（`<umbreon/umbreon.hpp>`）:
  `bool oidnDenoiserAvailable(const std::string& workerPath = "")` /
  `void shutdownOidnDenoiser()`。
- `FrameResult::denoiserUsed` / `pt1DenoiserUsed`（0=None, 1=Atrous, 2=OIDN）。

## 3. ビルド統合（cuemol2 リポジトリ）

### 3.1 `src/cmake/umbreon.cmake`

- **`find_package(OpenImageDenoise 2 CONFIG REQUIRED)` を削除**する。これは旧 umbreon が OIDN を
  PUBLIC リンクしていた名残で、新 umbreon は OIDN シンボルを一切持たない（worker だけがリンクする）。
  併せて先頭コメントの OIDN 段落を書き換える。
- `find_package(umbreon CONFIG REQUIRED)` は不変。
- worker の staging を追加（`umbreon.cmake` 内に置く。`ENABLE_UMBREON` ゲートと `if(TARGET)` 判定が
  1 ファイルに集まる）:

```cmake
# umbreon が OIDN 付きでビルドされていれば、worker を libcuemol2 の <prefix>/bin へ
# 再インストールする（blendpng と同じ場所。パッケージング/実行時解決の SSOT にする）。
if(TARGET umbreon::umbreon_oidn_worker)
  install(PROGRAMS $<TARGET_FILE:umbreon::umbreon_oidn_worker>
    DESTINATION bin)
  message(STATUS "umbreon OIDN worker: staged to <prefix>/bin")
else()
  message(STATUS
    "umbreon built without OIDN worker; GI denoise falls back to a-trous")
endif()
```

> `install(PROGRAMS <genex>)` は CMake >= 3.14。cuemol2 が 3.21+ なら
> `install(IMPORTED_RUNTIME_ARTIFACTS umbreon::umbreon_oidn_worker RUNTIME DESTINATION bin)`
> がより宣言的。

### 3.2 `src/modules/rendering/CMakeLists.txt`

変更不要（`umbreon::umbreon` を PRIVATE リンクのまま。OIDN 依存は消えている）。

## 4. パッケージング（tritium）

### 4.1 `tritium/packaging/collect-cuemol2-runtime.sh`

blendpng ブロック（`$LIBCUEMOL2_ROOT/bin/blendpng` → `$RUNTIME_DEST/bin/`）を鏡写しに追加:

```sh
UMBREON_WORKER_EXE="umbreon_oidn_worker"
[ "$PLATFORM" = "win" ] && UMBREON_WORKER_EXE="umbreon_oidn_worker.exe"
UMBREON_WORKER_SRC="$LIBCUEMOL2_ROOT/bin/$UMBREON_WORKER_EXE"
if [ ! -f "$UMBREON_WORKER_SRC" ]; then
  echo "Error: umbreon_oidn_worker not found: $UMBREON_WORKER_SRC" >&2
  exit 1   # ENABLE_UMBREON ビルドでは必須（blendpng と同格）
fi
cp "$UMBREON_WORKER_SRC" "$RUNTIME_DEST/bin/"
echo "  umbreon_oidn_worker: $UMBREON_WORKER_SRC"
```

最終アサート（`assert_file "$RUNTIME_DEST/bin/$BLENDPNG_EXE"` の隣）にも 1 行追加する。
`ENABLE_UMBREON=OFF` ビルドを流す運用があるなら、環境変数ゲートで warn+skip にする。

### 4.2 `tritium/react-gui/electron-builder.yml`

変更不要。`extraResources` が `packaging/cuemol2-runtime/bin` → `Resources/cuemol2/bin` を
丸ごと写すので、worker も自動的に `Resources/cuemol2/bin/umbreon_oidn_worker` に入る。

## 5. ランタイムパス解決（tritium/react-gui）

> **重要**: パッケージ済みアプリでは helper は `Resources/cuemol2/bin/` にあり Electron 実行ファイルの
> 隣ではない。umbreon の exe 隣接探索は不発になるので、**JS で解決した明示パスを
> `RenderOptions::oidnWorkerPath` に渡す**のが一次機構（blendpng と同じ）。

- `src/main/ipcHandlers.ts` `getRenderBinaries()`（blendpng と同じ分岐に 1 項目追加）:

  ```ts
  // packaged:
  umbreonOidnWorker: path.join(res, 'cuemol2', 'bin', `umbreon_oidn_worker${exe}`),
  // dev:
  umbreonOidnWorker: root ? path.join(root, 'bin', `umbreon_oidn_worker${exe}`) : '',
  ```

- `src/renderer/worker/shared/renderTypes.ts` `RenderBinaries` に `umbreonOidnWorker: string` を追加。
- `src/renderer/contexts/RenderConfigContext.tsx` の merge（既定値 + ユーザ設定上書き）に 1 項目追加。
- `components/panes/settings/settingsConfig.ts` に `rendering.umbreonOidnWorker`（既定 `''`）を追加。
- `src/renderer/worker/server/services/renderBackends/UmbreonBackend.ts` で解決済みパスを
  レンダパラメータに載せて core へ渡す。

## 6. libcuemol2 プロパティ配線

- `src/modules/rendering/UmbreonSceneExporter.qif`: `giDenoise` の隣に
  `property string oidnWorkerPath => m_oidnWorkerPath;` を追加（mcwgen が wrapper 生成）。
  `UmbreonSceneExporter.cpp` の member 既定値は `""`。
- `UmbreonSceneExporter.cpp` `setupContext()`: `prm.oidnWorkerPath = m_oidnWorkerPath;`。
- `UmbreonRenderParams` に `oidnWorkerPath` フィールド追加。
- `UmbreonDisplayContext.cpp` `buildSceneAndOptions()`（`opt` 設定域）:
  `opt.oidnWorkerPath = prm.oidnWorkerPath.c_str();`。

## 7. `OIDN_MAX_RENDER_AREA` キャップの撤去（検証ファースト）

`UmbreonDisplayContext.cpp` の定数 `OIDN_MAX_RENDER_AREA`（6MP）と `renderArea` ゲートは、
worker 分離でクラッシュ原因が消えるため撤去できる。ただし**大判で worker が正常動作することを
実測してから**独立コミットで撤去する（revert 可能に保つ）。撤去後は `prm.giDenoise` を無条件に
`opt.pt1Denoise = true` へマップする。umbreon 側では 4K（285MB リージョン）を実測済み。

## 8. UI / 診断

- 設定画面: `oidnDenoiserAvailable(resolvedPath)` で有効性を表示（true なら worker は warm 維持 =
  初回デノイズが速い。メモリを返したいときは `shutdownOidnDenoiser()`）。
- レンダ後: `FrameResult::pt1DenoiserUsed`（CueMol の GI 経路が実際に使うのはこちら。`giDenoise` は
  `pt1Denoise` に写像される）をレンダログへ（2=OIDN / 1=a-trous fallback / 0=off）。
- **disabled ラッチの解除**: 明示パスでの起動失敗は「そのパスで disabled」ラッチが立ち、同一パスの
  再 probe は静かに false を返す。パス設定を直した後は別パスでの再実行か `shutdownOidnDenoiser()` で
  解除する。

## 9. レガシー uxp_gui / dev モード

- **uxp_gui**: GreD ディレクトリ = 実行ファイル隣接なので、worker を `<prefix>/bin` に置くだけで
  既定探索で自動発見（`oidnWorkerPath` 空で可）。pref による明示上書きも可能。
- **dev (tritium)**: `getRenderBinaries()` の dev 分岐（`LIBCUEMOL2_ROOT/bin/...`）が本線。
  代わりに env `UMBREON_OIDN_WORKER=$LIBCUEMOL2_ROOT/bin/umbreon_oidn_worker` でも解決できる。

## 10. プラットフォーム注意

- **macOS 署名**（`cuemol2/docs/migration/adr/ADR-0030` 参照）: worker も個別に Developer ID 署名 +
  notarization + hardened runtime。ホストからの spawn には
  `com.apple.security.cs.disable-library-validation` entitlement が必要になり得る（blendpng/povray と同様）。
- **Windows**: worker は static OIDN リンクで自己完結のはず。`dumpbin /dependents umbreon_oidn_worker.exe`
  で system DLL のみか検証する。
- **共有メモリ**: umbreon は POSIX でメモリマップファイル、Windows で pagefile-backed の
  `windows_shared_memory` を使う。どちらもサイズ上限は実用上なし（4K = 285MB を実測済み。
  process-isolation.md §4 の「数十 MB 上限」懸念への回答）。一時ファイルはクライアントが破棄する。

## 11. チェックリスト（新規環境での確認）

1. `umbreon::umbreon_oidn_worker` が export に含まれる（`find_package(umbreon)` 後 `if(TARGET ...)` が真）。
2. libcuemol2 install 後、`<prefix>/bin/umbreon_oidn_worker[.exe]` が存在。
3. パッケージ後、`Resources/cuemol2/bin/umbreon_oidn_worker[.exe]` が存在。
4. `oidnDenoiserAvailable(resolvedPath)` が true を返す。
5. GI デノイズ描画後、`FrameResult::pt1DenoiserUsed == 2`。worker を殺すと次描画で `== 1`（a-trous）。
6. アプリ終了後に worker プロセスと一時ファイルが残らない。
