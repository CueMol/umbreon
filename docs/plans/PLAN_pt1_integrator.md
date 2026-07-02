# 実装指示書: パストレース間接光インテグレータ `pt1` の追加(最小実験)

## 0. 目的と背景

このリポジトリには、メッシュを読み込み irradiance cache で diffuse GI をレンダリングする機能が既にある。
本タスクでは、それと**並存する**新しいインテグレータ `pt1` を追加する:

> Embree + TBB による「間接 diffuse 1バウンス・8spp・半解像度・OIDN デノイズ」のパストレーサ。
> 目標性能: 1920x1080 で数十秒以内(マルチコア CPU)。既存 irradiance cache との品質・時間の A/B 比較が最終ゴール。

### 非目的(やらないこと)
- 既存の irradiance cache コードの削除・リファクタリング・挙動変更。**一切触らない**。
- multi-bounce(2バウンス以上)。ループを延長できる構造にはするが、本タスクでは 1 バウンス固定。
- スペキュラ / 屈折 / ボリューム。diffuse (Lambert) のみ。
- GPU 対応。

### 全体の進め方(重要)
- 下記 Phase 0〜6 を**順番に**実施する。各 Phase の「受け入れ基準」を満たしてから次に進む。
- 各 Phase 完了ごとに独立した commit を作る。commit message に Phase 番号を含める。
- リポジトリの実態(クラス名・ファイル構成)がこの指示書の想定と食い違う場合は、**指示書の意図を優先して既存構造に適合させる**こと。判断に迷う差異があれば Phase 0 の報告に書き、作業を止めて確認を求める。
- すべての放射量は **linear 色空間**で扱う。sRGB 変換 / トーンマップは最終出力の直前のみ。既存パイプラインの規約を Phase 0 で確認し、それに合わせる。

---

## Phase 0: リポジトリ調査とビルド統合

### 0-1. 調査タスク
コードを読み、以下を `docs/pt1_survey.md` にまとめて報告する(各項目: 該当ファイル・型名・要点を数行):

1. メッシュの内部表現(頂点/インデックス配列の型、頂点法線の有無、マテリアル=albedoの持ち方)
2. カメラモデルと primary ray の生成方法
3. 光源の表現(点光源 / 面光源 / 環境・スカイの有無)と直接照明の評価コード
4. 既存 irradiance cache インテグレータの呼び出し口(どこで分岐すれば新インテグレータを差し込めるか)
5. 画像バッファの形式と出力(EXR/PNG、linear か sRGB か、トーンマップの位置)
6. 既存の並列化の有無(スレッド、OpenMP 等)とビルドシステム(CMake か否か)

### 0-2. 依存ライブラリの導入
- **Embree 4**、**oneTBB**、**OpenImageDenoise (OIDN) 2.x** を導入する。
- CMake なら `find_package(embree 4 REQUIRED)`, `find_package(TBB REQUIRED)`, `find_package(OpenImageDenoise REQUIRED)` を基本とし、見つからない場合のインストール手順(apt / brew / vcpkg / 公式バイナリ)を `docs/pt1_survey.md` に記載する。FetchContent での Embree ビルドは重いので行わない。
- 新規コードは `src/pt1/` 以下(または既存慣習に合わせた場所)にまとめ、既存ファイルへの変更はインテグレータ分岐と CLI 追加の最小限に留める。

### 0-3. 疎通テスト
- 単一三角形を Embree シーンに登録し、`rtcIntersect1` でヒット/ミスを確認する最小ユニットテストを追加する。
- **Embree 4 の API** を使うこと(`RTCRayQueryContext` 等)。Embree 3 の `RTCIntersectContext` 系 API と混同しない。

### 受け入れ基準
- `docs/pt1_survey.md` が上記 6 項目を網羅している。
- 依存 3 ライブラリ込みでビルドが通り、三角形交差テストが pass する。

---

## Phase 1: シーンの Embree アップロードと G-buffer

### 1-1. ジオメトリ登録
- 全メッシュを `RTC_GEOMETRY_TYPE_TRIANGLE` として登録。可能なら `rtcSetSharedGeometryBuffer` で既存の頂点/インデックス配列を共有し、コピーを避ける。
- シーン作成時に `rtcSetSceneFlags(scene, RTC_SCENE_FLAG_ROBUST)` を指定(watertight 交差)。
- `rtcCommitScene` の所要時間を計測ログに出す。

### 1-2. Primary ray パスと AOV
1 ピクセルにつき primary ray を 1 本(ピクセル中心)撃ち、以下の full-res AOV バッファ(float の平面配列)を埋める:

- `position` (world, float3)
- `normal` (シェーディング法線=頂点法線を補間・正規化。無ければ幾何法線。world 空間, float3)
- `albedo` (float3, [0,1])
- `depth` (カメラからの距離, float)
- `hitFlag` (ミス判定)

法線の向き: `dot(N, -rayDir) < 0` なら N を反転して視線側に揃える(幾何法線・シェーディング法線とも)。

### 1-3. 自己交差対策(以後すべてのセカンダリ/シャドウレイに適用)
- レイ原点をヒット点から **幾何法線方向に `eps = 1e-4 * sceneScale` だけオフセット**する(`sceneScale` = シーン AABB の対角長)。加えて `ray.tnear = 0`, `ray.tfar = FLT_MAX`(シャドウレイは光源距離 − 2*eps)。
- この処理は関数 `offsetRayOrigin(p, ng)` として 1 箇所に実装し、全所から呼ぶ。

### 受け入れ基準
- `--integrator pt1 --aov` 等で AOV を EXR(なければ PNG)に出力できる。
- normal / depth の可視化画像が、既存レンダラの一次可視性(シルエット・面の向き)と目視で一致する。

---

## Phase 2: 直接照明

- 既存の直接照明評価コードが再利用できるなら再利用する(Phase 0-1 の調査結果に従う)。
- 再利用できない場合の最小実装: 光源を 1 サンプル選び、面光源なら面上の一様サンプル点へ、点光源なら位置へ **シャドウレイ(`rtcOccluded1`)** を撃つ。寄与は標準の NEE 式(BRDF ρ/π × 幾何項 × 可視性 ÷ pdf)。
- **スカイ(環境光)は直接照明パスに含めない。** スカイは Phase 3 の gather 側で扱う(二重計上防止のため。この分担を `docs/pt1_design.md` にコメントとして明記すること)。
- 出力: full-res の `directIrradiance` バッファ(float3)。ここでは「BRDF を掛ける前の照度 E_dir」を保存する(最終合成で ρ/π を掛ける)。既存コードが radiance ベースならその規約に合わせ、どちらの規約かをコードコメントに明記する。

### 受け入れ基準
- 直接照明のみの画像が既存レンダラの直接照明成分と目視で一致する(影の位置・明るさ)。

---

## Phase 3: 間接 1 バウンス gather(まず full-res で正しさ優先)

**この Phase が本体。半解像度化は Phase 5 まで行わず、まず full-res で数値的正しさを確立する。**

### 3-1. 推定量(この式の通りに実装する)

シェーディング点 x における間接照度(スカイ寄与込み)を、cosine 重み付き半球サンプリングで推定する:

```
E_gather(x) = ∫_hemisphere L_in(x, ω) cosθ dω

cosine サンプリング: pdf(ω) = cosθ / π
⇒ 推定量: E_gather ≈ (π / N) * Σ_{i=1..N} L_i        (N = spp, 既定 8)
```

各サンプル方向 ω_i の `L_i` は:

- **ミス(シーンに当たらない)**: `L_i = L_sky(ω_i)`。まず一様スカイ `L_sky = 定数`(CLI で指定、既定 1.0)で実装。動作確認後、簡単な勾配スカイ `L_sky = lerp(L_horizon, L_zenith, max(ω.y,0))` を `--sky gradient` で選べるようにする。
- **ヒット(点 y、法線 N_y、アルベド ρ_y)**:
  `L_i = (ρ_y / π) * E_direct(y)`
  ここで `E_direct(y)` は Phase 2 と同じ NEE(光源 1 サンプル + シャドウレイ)を y で評価したもの。
  **y での emission(光源面の自己発光)は加えない。** 光源から x への直接光は Phase 2 で計上済みであり、加えると二重計上になる。この理由をコードコメントに書くこと。

### 3-2. サンプリングと乱数
- cosine 半球サンプリング(接空間で `d = (r cosφ, r sinφ, sqrt(1-u1))`, `r = sqrt(u1)`, `φ = 2π u2`、シェーディング法線基準の正規直交基底で world へ変換)。
- 乱数は **PCG32** をピクセルごとにローカル生成する。seed は `hash(pixelX, pixelY, frameSeed)` で決定的にする(再現性のため)。**スレッド間で RNG を共有しない。**
- ガード: `dot(ω_i, N_geom) <= 0` になったサンプルは寄与 0 として捨てる(シェーディング法線と幾何法線の乖離対策)。

### 3-3. 並列化
- TBB `tbb::parallel_for` で **16x16 タイル**単位に分割(`blocked_range2d` 可)。タイル内は逐次。
- 共有状態への書き込みはタイル自身のピクセル範囲のみ。ロック不要の構造にする。

### 3-4. 出力と合成
- `indirectIrradiance` バッファ(float3, full-res)に E_gather を保存。
- 最終色: `color = (albedo / π) * (directIrradiance + indirectIrradiance) * π` …ではなく、規約を 1 つに統一すること。推奨規約: **バッファには照度 E を保存し、最終合成で `color = albedo * (E_dir + E_gather) / π * π = albedo * (E_dir + E_gather)`**(Lambert の出射輝度 L_o = (ρ/π)·E に π·(カメラ側の規約) が掛かるかは既存パイプラインに合わせる)。既存レンダラと同一シーン・直接光のみで輝度が一致することを基準に規約を確定し、`docs/pt1_design.md` に記録する。

### 受け入れ基準(数値テスト必須)
1. **平面テスト**: 一様スカイ `L_sky = 1`、光源なし、上向きの床平面 1 枚のシーンで、床中央の `indirectIrradiance` が **π (≈3.14159) に対し誤差 1% 以内**(spp=1024 で確認するテストを追加。8spp 時は分散が大きくてよい)。
2. **Cornell box テスト**: 白い箱 + 赤・緑の側壁 + 面光源のシーンで、白い床・天井に赤/緑の color bleeding が目視確認できる。テストシーンが無ければ簡単な OBJ を `tests/scenes/` に生成して追加する。
3. 8spp・full-res・1080p の実行時間をログ出力する(この時点で数十秒〜1分程度なら正常)。

---

## Phase 4: OIDN デノイズ

- デノイズ対象は **`indirectIrradiance` バッファのみ**。直接光・albedo はノイズレスなので触らない。
- OIDN の使い方:
  - `oidnNewDevice(OIDN_DEVICE_TYPE_CPU)` → `oidnCommitDevice`
  - filter type `"RT"`、`color` に indirectIrradiance、補助バッファとして `albedo` と `normal` を渡す。
  - `oidnSetFilterBool(filter, "hdr", true)` を必ず設定(入力は linear HDR)。
  - バッファは `OIDN_FORMAT_FLOAT3`、行ピッチに注意(パディング無しの密配列なら byteStride は 0 指定でよい)。
- **前処理チェック**: normal は正規化済み(ミスしたピクセルは (0,0,0) でよい)、albedo は [0,1]、indirect に NaN/Inf があればゼロ化してから渡す。NaN 混入は OIDN 出力全体を壊す。
- CLI `--denoise on|off`(既定 on)。off 時は素通し。

### 受け入れ基準
- 8spp のノイズ画像がデノイズ後に滑らかになり、コーナー・接地部の暗まりと color bleeding が保持されている(before/after の PNG を `outputs/` に保存して確認)。
- デノイズ時間をログ出力(1080p で数秒以内が目安)。

---

## Phase 5: 半解像度 gather + joint bilateral upsampling

### 5-1. 半解像度化
- 間接 gather(Phase 3)と、その入力になる位置/法線/albedo の取得を **W/2 × H/2** で行う。実装は「半解像度用に独立して primary ray を撃つ」(各 half ピクセル中心 = full-res の 2x2 ブロック中心)。full-res G-buffer の流用より単純で、バグが少ない。
- OIDN デノイズ(Phase 4)は **半解像度のまま**適用する(guide バッファも半解像度)。デノイズ後にアップサンプル。

### 5-2. Joint bilateral upsampling
full-res ピクセル f ごとに、対応する half-res の近傍 2x2 ピクセル h∈{4近傍} を、full-res の normal/depth をガイドに重み付き平均する:

```
w(h) = w_bilinear(h) * w_n(h) * w_z(h)
w_n = max(0, dot(N_f, N_h))^32
w_z = exp( -|z_f - z_h| / (0.02 * z_f + 1e-6) )
E_f = Σ w(h)·E_h / Σ w(h)
```

- **フォールバック**: `Σ w(h) < 1e-6` の場合(エッジで全近傍が不一致)、最も近い half ピクセルの値をそのまま使う。
- 係数(指数 32、深度スケール 0.02)は定数として括り出し、CLI から変更可能にしておく。

### 受け入れ基準
- Cornell box で full-res 版(Phase 4 出力)と目視比較し、オブジェクト輪郭での光漏れ・にじみが無い。
- full-res 版に対する PSNR を計測ログに出す(参考値。閾値は設けない)。
- 1080p・8spp・半解像度・デノイズ込みの合計時間が **60 秒以内**(目標 10〜20 秒)。ボトルネックが gather 以外(例: デノイズ、I/O)にある場合は原因をログで特定して報告する。

---

## Phase 6: CLI・計測・A/B 比較ハーネス

### 6-1. CLI(既存の引数体系に合わせて追加)
```
--integrator {cache|pt1}     既定: cache(既存挙動を変えない)
--spp N                      既定: 8
--indirect-res {full|half}   既定: half
--denoise {on|off}           既定: on
--sky {uniform|gradient}     既定: uniform
--sky-radiance R G B         既定: 1 1 1
--seed N                     既定: 0
```

### 6-2. 計測
以下を段階別に計測し、標準出力と `outputs/timing.json` に書く:
`bvh_build / primary / direct / gather / denoise / upsample / total`

### 6-3. 比較スクリプト
`scripts/compare_integrators.sh`(または .py)を追加:
1. 同一シーン・同一カメラで `cache` と `pt1` を実行
2. 画像 2 枚と timing を `outputs/compare_<scene>_<date>/` に保存
3. 実行時間の比を表示

### 受け入れ基準
- 代表シーン(既存リポジトリで irradiance cache が 5 分かかる規模のもの、無ければ最大のテストシーン)で比較スクリプトが完走し、画像 2 枚と時間比が得られる。

---

## 落とし穴チェックリスト(実装中に随時確認)

- [ ] Embree 4 API(`RTCRayQueryContext`)。Embree 3 のコード例を写さない。
- [ ] `rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID` でミス判定。`rtcIntersect1` 前に `geomID` を `RTC_INVALID_GEOMETRY_ID` に初期化する。
- [ ] シャドウレイの `tfar` は光源までの距離 − 2·eps(光源面自体に当たって「遮蔽」と誤判定しない)。
- [ ] 自己交差: 全レイで `offsetRayOrigin` を使用(Phase 1-3)。
- [ ] cosine サンプリングの pdf と π の係数(推定量は Phase 3-1 の式の通り。cosθ と pdf が相殺して π のみ残る)。
- [ ] 二重計上の 2 大源: (a) gather ヒット点での emission 加算、(b) スカイを直接照明とgatherの両方で加算。どちらも禁止(Phase 2, 3-1 参照)。
- [ ] RNG のスレッド共有禁止・ピクセル決定的 seed(Phase 3-2)。
- [ ] OIDN: `hdr=true`、NaN 除去、normal 正規化(Phase 4)。
- [ ] linear/sRGB の混同: バッファはすべて linear。PNG 出力時のみ変換。
- [ ] TBB のタイル外書き込み無し(データ競合)。可能なら TSan で 1 回実行して確認。

## 完了報告に含めるもの
1. 各 Phase の受け入れ基準の結果(数値テストは実測値を記載)
2. Cornell box と代表シーンの最終画像(cache / pt1 両方)
3. timing.json の要約(段階別時間、cache との時間比)
4. 既知の制限・次の改善候補(例: multi-bounce 化はどのループを延長すればよいか、を 1 段落で)
