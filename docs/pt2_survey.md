# pt2 設計サーベイ: pt1 の次に載せる現代的 GI 手法の選定

pt1(当時 `--gi on` の既定)の後継として何を実装しうるかのサーベイ。
**2026-07 追記: pt2 は既定インテグレータに昇格し、pt1 は凍結アンカーになった**
(`--integrator pt1` で明示選択)。
候補手法を分子グラフィクスというドメインと本リポジトリの制約に照らして評価し、
段階的なロードマップを提案する。**本書は設計文書であり、実装の約束ではない** —
各段の go/no-go は §5 の判定基準で個別に決める。

対になる文書: [pt1_design.md](pt1_design.md)(継承する規約)、
[pt1_tuning.md](pt1_tuning.md)(現行の性能・品質基準)。
`docs/plans/surface-irradiance-cache-rev2.md`(adaptive irradiance cache 案、未着手)は
本書をもって supersede する — cache 系はもう追わない(§2 却下リスト参照)。

## 1. スコープと制約

### 目標(2026-07 ユーザー確認)

1. **多バウンスの実質無料化** — bounce ≥ 2 を新規レイでなく既計算の再利用で賄う
2. **同コストでの品質向上** — 同じレイ数でノイズを大幅に減らす(= denoise 前の入力を
   綺麗にし、denoise 後のディテール保持を上げる)
3. **表現力の拡張** — pt1 にない見た目の追加(glossy 間接光、emissive 光源など)
4. アニメーション(連番フレーム)出力も将来視野に入れる

### 制約

- **glass / 水の屈折・caustic はニーズなし**。透過は既存の transparency(直進減衰)で足りる
- **計算量は同オーダー**。増加は許容(〜2-3x 程度まで)、10x は NG
- 分子シーンの特性を前提としてよい: 開けた凸主体のジオメトリ(gather hit 率 ~17%)、
  distant light 2 灯(key + fill)、orthographic が主、diffuse 支配、白背景 + fog

### pt1 から継承する規約([pt1_design.md](pt1_design.md) より。違反は設計ミス)

- **エネルギー**: `E_stored = mean(L_i) = E_true/π`。`1/π` はどこにも掛けない。
  composite は `color += giIntensity * giReflectance * E`
- **sky は gather の miss 項のみ**。direct pass に sky はなく、GI-eligible ヒットでは
  constant ambient をゼロにする(二重計上ガード)
- **bounce 頂点は NEE の反射光のみ**を加算する。ただし emission については §2-F 参照
  (direct pass は emissive 面→他面の輸送を行っていないため、bounce 頂点での emission
  加算は二重計上に**ならない** — pt1 が読まないのは単に未実装)
- **決定論**: tea2 ハッシュのみ、per-pixel seed、スレッド数不変、run-to-run bit-exact。
  新手法は候補ごとに「これをどう保つか」を明記する(§2 各項)
- **Route A**: GI が真値であり、AO を composite に混ぜない(rev2 で確定済み。再訪しない)

### 再提案禁止(検討・却下済みの経緯があるもの)

AO 主体の depth-cue(Route B)/ `1/π` の `giIntensity` への折込み / constant ambient + AO
乗算 / PCG32(tea2 で決着)/ env-dome(`--env-light`)と GI の併用(sky 二重計上)。

## 2. 候補手法の評価

各候補: 仕組み / 期待効果 / コスト(draft 基準: gather 0.29s・denoise 1.08s・全体 1.94s、
`1ab0_scene1` 1200²)/ 決定論 / OIDN との相互作用 / 実装規模。

### A. ReSTIR-GI 空間リサンプリング — 品質の主役(目標②)

**仕組み**: pt1 の gather は各ピクセルが完全に独立で、隣のピクセルが見つけた「良い」
サンプル点(明るい間接光をもたらした bounce-1 ヒット点)の情報を一切共有しない。
ReSTIR-GI(Ouyang et al. 2021、理論的基盤は GRIS: Lin et al. 2022)はこれを
reservoir resampling で共有する:

1. **候補生成**: 各 gather ピクセルが従来通り少数(例 4)の bounce-1 サンプル点
   `{hit 位置 x_s, 法線 n_s, その点の outgoing radiance L_s}` を生成し、weighted
   reservoir に保持
2. **空間再抽選**: 固定パターンの近傍(例: 半径 8-16px の低食い違い量オフセット
   5-8 点)から reservoir を取り寄せ、target pdf(自ピクセルでの寄与
   `L_s · cosθ`)で再抽選。ジオメトリが違いすぎる近傍(法線・深度の閾値)は棄却
3. reservoir のサンプル点を自ピクセルの立体角へ **Jacobian 補正**して E に計上

サンプル点の生成コスト(レイトレース)は 1 回で、再抽選は G-buffer 上の算術のみ。
近傍 8 点 × 2 ラウンドの反復で**実効 spp が ~8-20 倍**になる。これは「spp を増やす」
のと違い denoise 前の入力が構造的に綺麗になるので、OIDN が潰しがちな細部
(ポケット内の暗部グラデーション)の保持がよくなる。

**バイアスの扱い**: 近傍のサンプル点は自ピクセルから可視とは限らない。
- **biased 版(既定案)**: 可視性を再チェックしない。誤差は「影がわずかに漏れる」
  方向で、OIDN 後にはほぼ視認不能というのが実運用の定説
- **unbiased 版(フラグ)**: 採択サンプルへ shadow ray 1 本の可視性再チェック。
  gather コスト +50-100% だが全体では×2 未満に収まる(コスト制約内)

**コスト**: 候補生成は現行 gather と同じ。再抽選パスは G-buffer 演算のみで
~0.05s オーダー。**gather ×1.3-1.5(biased)、×2(unbiased)**。denoise は不変。
同 PSNR なら spp を 1/4〜1/8 に落とせる余地があり、実質高速化にも転用可能。

**決定論**: 成立する。候補生成(pass 1)を完了させてから再抽選(pass 2)を行う
2-pass 構成にすれば、pass 2 は完成済みバッファの純関数になる。近傍パターンは
per-pixel seed から決定的に生成。tile 並列でも読むだけなので順序非依存。
pt1 の 16×16 tile 並列 + tea2 seed の枠組みがそのまま使える。

**OIDN との相互作用**: 入力ノイズが下がるのは純益だが、ReSTIR 特有の
「近傍で同じサンプルが勝ち続けることによる低周波のブロッチ」は OIDN が
苦手とする形なので、再抽選ラウンド数と近傍半径は 1ab0 系シーンで実測調整が必要。
per-sample firefly clamp(`--pt1-clamp`)は reservoir の重みに移設する。

**実装規模**: 中。`pt1_gather.hpp` の gather 本体はほぼ流用(サンプル点を E に
畳み込む前に reservoir へ格納する分岐を足す)。新規は reservoir バッファ
(gather グリッド × ~32B)と再抽選パス 1 ファイル。

### B. サンプラー刷新: Owen-scrambled Sobol + blue-noise シード(目標②)

**仕組み**: 現行の LD は Hammersley + per-pixel Cranley-Patterson shift
(`pt1_gather.hpp`)。これを (1) 次元間の相関がより良い **Owen-scrambled Sobol**
(Burley 2020 の hash-based Owen scrambling は tea2 と同系の整数ハッシュで実装でき、
テーブル不要・決定論そのまま)に置換し、(2) ピクセル間のシード配置を白色ノイズから
**blue-noise マスク**(Heitz & Belcour 2019 系)にする。誤差の総量は変わらないが
分布が高周波側へ移り、**OIDN(と人間の目)が最も除去しやすい形**になる。

**コスト**: ×1.0(ハッシュ数回/サンプル)。**実装規模: 小**(サンプラー関数の差替えと
A/B 比較のみ)。決定論: 完全維持。アニメ(§G)まで見るなら spatiotemporal blue noise
(STBN)へ拡張でき、フレーム間のちらつき低減に直結する。
費用対効果が最も高く、**他のどの候補とも直交**する(必ず最初にやる)。

### C. 分散適応 spp(目標②、コスト上限付き)

**仕組み**: 2-pass。pass 1 で基本 spp(例 8)を撃ち、per-pixel の輝度分散を推定。
分散が閾値を超えたピクセル(≒凹部・接触影)だけ pass 2 で追加 spp(例 +24)を撃つ。
既存の `--aa adaptive`(flag → 追加サンプル)と同じ 2 相決定論パターンであり、
リポジトリの文化(`--ao-res out` の patch 機構、edge-patch)にもよく馴染む。

**コスト**: ×1.0〜1.5 で**上限が設計時に固定できる**(追い gather の spp とピクセル
比率で決まる)。分子シーンでは高分散域は面積の 10-20% に集中する(凹部)ので、
一様に spp を倍にするより遥かに効率がよい。

**決定論**: pass 1 完了後に flag マスクを確定 → pass 2 は flag の純関数。
adaptive-AA と同一の論法で bit-exact。**実装規模: 小-中**。
A(ReSTIR)と併用可能だが、まず A 単体の実測を見てから重ねるか決める。

### D. screen-space radiance reuse による多バウンス無料化(目標①)

**先に正直な注意**: 実測では pt1 の traced 多バウンスの限界コストは小さい
(gather hit 率 ~17% の開けたシーンでは、bounce 2 の継続レイは全レイの 2 割弱。
実測でも gather/denoise 比は bounce 項なしの `0.03·spp` にフィットする)。
つまり「bounce 2 を無料化」の絶対的な節約額は限定的。**本命の価値は
bounces=∞(収束するまでの全次数)を有界コストで既定にできること**にある —
深いポケットの明るさは 3 次以降の相互反射で立ち上がる領域があり、そこは現行の
`--gi-bounces 2-3` 打ち切りでは系統的に暗い。

**仕組み**(Lumen の final gather と同型の反復):

1. 通常の bounce-1 gather を実行し、gather グリッドに `E^(1)` を得る
2. 反復ラウンド k: gather レイの hit 点をカメラへ投影し、gather グリッドの
   G-buffer と深度・法線が一致すれば、その点の間接光を**トレースせず**
   `albedo · E^(k-1)` の参照で得る(screen-space lookup)。
   不一致(画面外・遮蔽・裏面)なら従来の path 継続へフォールバック
3. 収束(‖E^(k) − E^(k-1)‖ が閾値未満、実用上 2-3 ラウンド)まで反復

凹部ほど hit 率が高い = **最も多バウンスが効く場所で最も lookup が当たる**という
好相性がある。orthographic 主体の分子レンダでは「gather レイの hit 点が画面内の
可視面」である率が高く、screen-space の弱点(画面外情報の欠落)が出にくい。

**コスト**: ラウンドあたり lookup 主体で **×1.05-1.1**。フォールバックの path 継続は
RR で有界。世界空間 hash-grid cache(SHaRC 系)なら画面外もカバーできるが、
並列 accumulate の決定論化(固定順序 reduction)という重い課題を持ち込むため、
**まず screen-space + フォールバックで足りるかを実測してから**判断する。

**決定論**: ラウンド間は完全バリア(E^(k-1) を読むだけ)なので 2-pass 論法で
bit-exact。**実装規模: 中**。hit 点の再投影は `tracePt1GBuffer` と同じカメラ基底
(`toPt1Basis`)を流用。

### E. glossy/specular 間接光(目標③)

**現状の gap**: direct pass は Blinn/Phong の解析ハイライトを持つが、
`Material.reflection` は **`reflection * background` を返すだけのフェイク**
(`shading.hpp:154-158`、レイなし)。gather も diffuse 専用(`pt1EvalVertex` は
`mat.diffuse × pigment` しか読まない)。つまり「分子表面への他の分子・自己の
映り込み」は現在一切描けない。

**仕組み**: specular/reflection を持つマテリアルの一次ヒットで、GGX importance
sampling による reflection gather を少数 spp(4-8)追加する:

- POV の `roughness`/`phongSize` → GGX α への変換表を用意(direct pass の
  `blinnExp` マップと整合させ、ハイライトと映り込みのシャープさを一致させる)
- 反射レイのヒット点は **diffuse 評価のみ**(direct NEE + sky miss)。
  glossy→glossy の連鎖はしない(caustic 系の輸送は対象外、1 段で十分)
- composite は diffuse E と**別項**: `color += reflection · F(θ) · E_spec`。
  `giReflectance`(diffuse 反射率)は掛けない
- E_spec バッファも OIDN に通す(diffuse E と同じ guide が使える)

**コスト**: glossy ピクセル比例。分子表面全面が specular を持つ場合でも
+4spp ≒ gather ×1.5 程度。glossy マテリアルが無いシーンでは**完全にゼロ**
(マテリアルスキャンでパスごと skip)。**決定論**: 通常の gather と同じ。
**実装規模: 中**(BRDF サンプラー + 別バッファ + composite 項 + α 変換表)。

### F. emissive geometry を GI 光源に(目標③)

**現状の gap**: `Material.emission` は POV の `finish emission` からパース済みで、
direct pass の自己発光項として機能している(`shading.hpp:63-65`)。しかし
gather の bounce 頂点では読まれない — つまり「光る物体が周囲を照らす」輸送が
存在しない。direct pass は emissive→他面の輸送を行っていないので、
**bounce 頂点での emission 加算は pt1_design.md の二重計上ガードに抵触しない**
(ガードが禁じているのは「direct が既に払った光源→受光面」の再計上)。

**第 1 段(ほぼ無料)**: `pt1EvalVertex` で `mat.emission × pigment` を加算するだけ。
「光る ligand が結合ポケットを照らす」「蛍光ラベルの色が周囲に滲む」という
分子 viz 的に映える表現が spp 追加なしで手に入る。大きく暗い emitter
(発光面全体)ならこれで実用ノイズに収まる。

**第 2 段(必要になったら)**: 小さく明るい emitter は BRDF sampling だけでは
ノイズが強い。emissive 三角形リストへの NEE(RIS で数本に絞る)を追加する。
これは将来 many-light(光源多数)へ進む場合の入口を兼ねる。ReSTIR DI や
light tree は**光源 2 灯の現状では無意味**であり、この第 2 段が入って初めて
検討対象になる(却下リスト参照)。

**コスト**: 第 1 段 ×1.0。第 2 段は emitter 数と NEE 本数次第(+10-30%)。
**決定論**: 第 1 段は自明。第 2 段の RIS も per-pixel seed で閉じる。
**実装規模: 第 1 段は小**(数行 + テスト)、第 2 段は中。

### G. temporal 再利用 — アニメーション track(pt3 スコープ)

**なぜ pt2 に入れないか**: 現 API は `render(scene, opt)` が呼び出しごとに BVH を
再構築する stateless 設計で、フレーム間の履歴(前フレームの E、reservoir、
motion vector)を持つ場所がない。temporal 再利用は **RenderSession 的な
状態保持 API の新設**が前提であり、libumbreon の公開 API を再形成する
(CueMol 側の呼び出しコードと要調整)。integrator の改良と独立した大工事なので
トラックを分ける。

**pt3 での段階案**(記録として):

1. **カメラパス動画**(剛体シーン): BVH 再利用 + 前フレーム E の reprojection +
   指数移動平均の temporal 蓄積(深度・法線の不一致で棄却)。フレームあたり
   実効コスト ×0.5-0.7、かつフレーム間ちらつきが激減する
2. **ReSTIR temporal**: A の reservoir をフレーム間で持ち越す(ReSTIR 本来の形)。
   A を spatial で実装しておけば追加は reservoir の reprojection のみ
3. **MD トラジェクトリ**(変形ジオメトリ): motion vector の定義が非自明
   (原子対応から頂点速度は出せる)。ここまで来たら別途設計

**決定論**: run-to-run bit-exact は「同一フレーム列に対する決定論」に緩和される
(フレーム n の結果はフレーム n-1 に依存してよい)。B の STBN がここで効く。

### 却下リスト(理由付き)

| 手法 | 却下理由 |
|---|---|
| Path guiding (PPG/Vorba 系) | 難しい間接光(屋内・ほぼ caustic な経路)で回収する手法。開けた 1-2 bounce 支配の分子シーンでは SD-tree の学習 overhead が回収できない |
| Neural radiance cache (NRC) | オンライン学習が非決定的で GPU 前提。CPU/Embree + bit-exact 文化と根本的に非適合 |
| Radiance cascades | リアルタイム・完全動的シーン向けの償却構造(一定コストの多スケール probe)。オフライン静止画では複雑さに見合う利得がない |
| DDGI / probe grid / irradiance cache 系 | `surface-irradiance-cache-rev2.md` を本書で supersede。世界空間キャッシュは cache 案を止めたのと同じ spacing/leak/補間チューニングの泥沼を再導入する。per-pixel gather + 再利用(A/D)の方が同じ課題をピクセル精度で解く |
| Photon mapping / BDPT / VCM | caustic・SDS 経路向け。対象外の要件のためだけに複雑さとメモリを払うことになる |
| ReSTIR DI / light tree (many-light) | 光源が distant 2 灯では選択問題が存在しない。F 第 2 段(emissive NEE)導入後に emitter が増えたら再訪 |

## 3. アーキテクチャ方針

- **pt2 は新 integrator id**(`RenderOptions::giIntegrator = 2`)とし、pt1 は
  regression anchor として凍結する(cache=0 / pt1=1 / pt2=2)。評価コア
  (`pt1EvalVertex`、NEE、`environmentRadiance`、tea2/LD サンプラー)は
  `experimental/pt1/` から共有し、pt2 固有物(reservoir、反復 bounce、spec gather)
  は `experimental/pt2/` に置く
- **エネルギー・composite の契約は pt1 と同一**(§1)。E バッファの意味が変わらない
  ので、denoise / upsample / edge-patch / 進捗(`GiProgressBudget` のスライス機構)は
  そのまま乗る。新サブステージ(再抽選パス、反復ラウンド)はスライスを足すだけ
- **決定論は 2-pass 論法で守る**: 「バッファを完成させてから、次のパスはその純関数」
  を候補 A/C/D すべての基本形にする。tile 並列は読み取り専用参照に限る
- **検証**: `scripts/compare_integrators.sh` を pt1 vs pt2 比較に拡張し、
  `--quality ultra`(参照)との PSNR と wall time を各段で記録する。
  bit-exact 検証は `scripts/refactor_check.sh` に pt2 ケースを追加

## 4. 推奨ロードマップ

> **改訂 (2026-07-16、フェーズ1の実測を受けて)**: 当初ロードマップは下記の通り
> pt2.0 → 2.1 の順で実装したが、**pt2.1 (ReSTIR-GI spatial) は実測で不成立**
> (§4.1)。以降の方向はユーザー決定により **C 先行実験 → B' 適応 spp →
> A' full PT 化** に改訂(§4.2)。

### 4.1 フェーズ1の結果 (2026-07-16、branch feat/pt2-integrator)

| 段 | 結果 |
|---|---|
| **pt2.0** サンプラー + emissive | **採用**。pt1 と同等品質 (ao_test1_hi denoised で pt1 38.03dB / pt2 38.17dB vs spp=256 参照)、コスト ×1.0。emissive 輸送は新表現かつ full PT 化の部品 |
| **pt2.1** ReSTIR-GI spatial | **不採用 (既定 off、`--pt2-rounds 0`)**。生 E の PSNR (vs spp=256 参照): spp=8 で reuse なし 31-40dB に対し reuse あり 21-32dB と **7-9dB 悪化**。勝つのは spp=1 のみ (+0.8-1.8dB)。凹部の多いシーン (ao_test1_hi / densurf) ほどギャップが拡大。denoise 後は誤差が「凹部陰影の持ち上がり (明るバイアス)」として残り、ユーザー目視でもエッジ部ノイズを確認 |

**pt2.1 不成立の構造的理由**: spatial-only の reservoir は 1 ピクセルの spp
サンプルを 1 個に圧縮して情報を捨てる。参照実装 (GPU リアルタイム) はフレーム間の
temporal 蓄積 (M~20-30) でこれを補うが、静止画にはそれがない。LD 層化 ≥4spp +
OIDN という本リポジトリの前提では素の平均が優れた推定量だった。

**申し送り (ReSTIR コードの扱い)**: 実装 (`experimental/pt2/pt2_reservoir.hpp`,
`pt2_spatial.hpp`) は決定論検証済み・既定 off で残置。テストは書かない。
**唯一の将来価値は動画 (pt3 temporal) の土台** — 蓄積 M の領域では成立し得る。
pt3 の方向性が動画でなくなった時点で削除してよい (git 履歴 commit 1f15601 から
復元可能)。

### 4.2 改訂ロードマップ

| 順 | 内容 | 目標 | 根拠・条件 |
|---|---|---|---|
| **C** | **多バウンス先行実験**(半日): ao_test1_hi / densurf で traced bounces=2/4/8 の収束差と時間を実測し、pt2.2 (screen-space 再利用) の go/no-go を決める | ① | **完了 (2026-07-16) — 判定 NO-GO**、§4.3 参照 |
| **B'** | **分散適応 spp**(§2-C): 凹部(高分散域)のみ追加サンプル。Cycles/appleseed 型、adaptive-AA と同じ 2-pass 決定論 | ② | ReSTIR に代わる「同コストで品質」の本命。確実・小規模・A' と直交 |
| **A'** | **full PT 化**: (1) traced mirror reflection → (2) area light → (3) emissive NEE + MIS | ③ + 厳密さ | **3 段とも実装済み (2026-07-17)**: (1) reflection 面がミラーレイ 1 本で実ジオメトリを映す(フェイク reflection*background 置換)。(2) per-light area light (`DistantLight::angularRadius`) を直接光+gather NEE の両方でサンプリング(ユーザー確認: soft shadow で奥行き感が自然に)。**価値の序列に注意**(ユーザー指摘 2026-07-17): 本体は「間接光まで一貫した per-light 光源モデル」— 既存の `--light-radius` は直接光にしか効かず、gather NEE はハードのままで直接/間接の光源モデルが食い違っていた。API 利用者(CueMol)は angularRadius をセットするだけで両方が整合する。POV からの自動導出 atan(spread/40) は bench/検証用の便利機能にすぎない(実運用は API から直接 Scene を構築し、POV reader を経由しない)。(3) emissive 三角形の power-CDF NEE + balance-heuristic MIS — 小型高輝度 emitter で raw-E +32dB (16.0→48.2dB @spp8)、収束平均一致 0.015%(不偏)。glossy BSDF と CSG emitter NEE はフェーズ2で実装(§4.4) |
| pt3 | temporal / RenderSession(動画)。ReSTIR 基盤はここで再評価 | アニメ | 別トラック |

### 4.3 多バウンス先行実験の結果 (2026-07-16) — pt2.2 は NO-GO

計測 (spp=64 で打ち切りバイアスを分離、vs bounces=8、400²):

| bounces | ao_test1_hi | densurf | gather 時間 (AH) |
|---|---|---|---|
| 1 | 19.2 dB | 17.8 dB | 0.84s |
| 2 (high 相当) | 27.0 dB | 24.1 dB | 1.23s |
| 4 | 39.6 dB | 34.7 dB | 1.48s |
| 8 | 参照 | 参照 | 1.57s |
| 16 | 55.3 dB | 49.4 dB | 1.60s |

- **bounces=8 でほぼ完全収束**(vs 16 が 49-55dB、Russian roulette が経路を自然に
  打ち切るので時間も伸びない)。end-to-end (1200², draft 相当) では bounces=2→8 が
  **+7%**。
- **判定**: screen-space 再利用が節約できるのは gather の bounce≥2 分 = 全体の
  +7% のみ。go/no-go 基準「∞バウンスが bounces=2 比 +20% 以内」は**素の traced
  bounces=8 が既に満たす**ため、再利用機構を作る意義がない。
- **より重要な発見(ユーザー目視判定)**: bounces=8 は物理的には正しいが、相互
  反射が凹部の陰影を埋めて**奥行き感が失われる**(albedo 0.8 の表面では顕著。
  露出調整では輝度比が戻らない)。分子 viz の目的 — 凹部の陰影による形状知覚 —
  にとって、**bounces は収束させるべきパラメータではなく奥行き感を制御する表現
  ノブ**である: 1 = 最深の陰影 / 2-3 = POV radiosity 相当のバランス / 8 = 物理
  参照。既定は変更しない(pt2 既定 = 1、high プリセット = 2)。
- 帰結: 目標①「多バウンスの実質無料化」は解決済み扱いとする(欲しいときは
  `--gi-bounces 8` が +7% で買える。既定で欲しい動機が viz 目的では存在しない)。

**material 依存性の追試 (同日、ユーザー指摘による)**: 「明るくなりすぎ」は
albedo 0.8 の白表面に固有 — 全 texture を diffuse 0.5 に落とした変種では b1→b8 の
持ち上がりが半分以下 (+12 vs +25/255) に収まり陰影が保たれる。色滲みが弱いのは
輸送でなくシーン側の問題 — densurf の表面は純白 <1,1,1> で滲み源がゼロ。オレンジ
表面の変種では bounce を重ねると彩度が上がる (0.681→0.718、凹部の自己反射で色が
飽和) = 輸送は機能している。よって §4.3 の「bounces = 表現ノブ」は「高 albedo 白
表面では b8 で陰影が浅くなる」という条件付きの整理に精密化する。

### 4.4 フェーズ2 (2026-07-17、branch feat/pt2-phase2)

スコープ(ユーザー確定): **glossy GGX 間接反射(§2-E / pt2.3)+ CSG emitter NEE
(§2-F 第2段の残り)**。reader の罠の修正は見送り(検証シーンは per-block
インライン finish で回避)。

| 段 | 内容 |
|---|---|
| **pt2.3-E glossy** | `reflection` かつ `specular > 0` の finish は GGX ローブ(Heitz 2018 VNDF、height-correlated Smith、weight = G2/G1)で映り込みをぼかす。**α はハイライトと同じ写像**(ユーザー決定): e = blinnExp(roughness) = 1/roughness、α = √(2/(e+2))。α < 0.015 と `specular == 0` は stage-1 の完全ミラー 1 本パスを bit-exact 維持(ピクセル分割は per-pixel reflAlpha)。E_spec は full-res の別バッファ → OIDN(定数白 albedo guide + normal/position — OIDN は normal 単独 guide を拒否し、実 albedo guide は反射像を pigment 境界で edge-stop するため)→ `color += reflection · E_spec`(Fresnel なし、POV スカラー reflection の意味論)。フラグ: `--pt2-glossy on` / `--pt2-glossy-spp 8` / `--pt2-glossy-denoise on`。該当材なしシーンは全体スキップ(bit-exact、テストで証明) |
| **CSG emitter NEE** | emissive NEE を mesh 三角形 + REAL CSG 球(solid-angle cone サンプリング、1−cosθmax は sin²/(1+cos) のキャンセレーション安全形を NEE/MIS 両側で共有)+ capped 円柱(側面 + cap×2 の面積サンプリング)に拡張。open 円柱チェーン(primID 対応がチェーンビルダ再現を要する・ほぼ装飾)と fromEdge 装飾(pt1EvalVertex が吸収するため NEE 化すると片側推定)は BSDF-only のまま = 不偏。**三角形は CDF プレフィクスを維持**し、mesh-only シーン(refactor_check pt2/pt2em)は byte-identical(検証済み)。NEE 1 サンプルの tea2 消費は常に 2 draw(pick+point)で継続ストリームは draw 安定 |

検証: `task test` 全通過(pt2_render 26 checks: 決定論 run-to-run + スレッド数、
gate no-op bit-exact、α→0 のミラー縮退 bitwise、near-mirror 3%、VNDF spp8/64
一致 2%、CSG 球/円柱の NEE vs BSDF-only 収束一致 2%、spp8 で MSE 半減、fromEdge
除外)。`refactor_check.sh` 既存 10 ケース byte-identical。検証シーン:
`data/glossy_test.pov`(傾斜 glossy ランプ + 色付きプリミティブ)、
`data/emissive_csg_test.pov`(白壁 + 発光球/発光ボンド、reader が CSG finish の
emission を拾うことを確認済み: "0 triangle(s) + 4 CSG record(s)")。
目視判定はユーザーに委ねる(glossy の α 写像と αmin=0.015 が主調整ノブ)。

**保留 (reader の罠、2026-07-16 発見・修正保留)**: bench の mesh2 reader は
mesh2 ブロックの material を「ファイル中で最初に finish 付き texture が宣言された
もの」から採用する (`adoptMaterial`、`haveBlockMaterial_` はブロック終了時のみ
リセット) — そのブロックの texture_list と対応しない。CueMol 出力は全 finish が
同一なので実害が出ていないが、material を意図的に変える作業 (full PT の glossy
検証シーン等) で踏む。修正は理論上出力を変えうるため、pt1 凍結ゲートと調整の上で
別途行う。それまで material 実験は「全 texture を一括書き換え」で回避する。

## 5. go/no-go 判定基準

各段とも `1ab0_scene1` 1200²(+ 凹部の多い `1ab0_scene6_densurf` 系)で計測:

- **品質段(pt2.0/2.1)**: 等時間比較で ultra 参照に対する PSNR が pt1 draft 比 +2dB
  以上、または等 PSNR で時間 30% 減。加えて denoise 後の目視比較
  (ユーザー判定 — 自己判定はしない)
- **多バウンス段(pt2.2)**: bounces=∞ の結果が traced bounces=8 と PSNR 40dB 以上で
  一致し、コストが bounces=2 比 +20% 以内
- **表現段(pt2.3/F)**: 該当マテリアルなしのシーンで bit-exact(完全 skip の証明)、
  ありのシーンで目視判定
- 全段共通: `refactor_check.sh` の既存ケースが byte-identical(当時 pt2 は opt-in なので
  既定経路に影響しないこと)、スレッド数を変えて bit-exact、`task test` 全通過

## 6. 参考実装(2026-07 に URL・所在を実地検証済み)

用途は**アルゴリズム理解のための参照**(コード移植ではない)なので、ライセンスは
選定基準にしない。順位は資料としての読みやすさ・完全性・umbreon との構造的な
近さのみで付けている。

### pt2.0-B: サンプラー(Owen-scrambled Sobol + blue-noise)

| 参照 | 所在・要点 |
|---|---|
| **pbrt-v4** | `src/pbrt/util/lowdiscrepancy.h`(`FastOwenScrambler`)+ `src/pbrt/samplers.h`(`ZSobolSampler` = Ahmed & Wonka の blue-noise Sobol)。(pixel, sample index, seed) の純関数でありスレッド数不変が構造的に成立 — umbreon の決定論モデルと同型。本(pbr-book.org 4ed §8.7)が行単位の解説 |
| **Blender Cycles** | `src/kernel/sample/sobol_burley.h` + `pattern.h`。テクスチャ不要の blue-noise ピクセル配置(Morton 曲線 + base-4 Owen scramble、~20行)。Blender 4.2 で blue-noise が既定化された実運用実績 |
| **Burley JCGT 2020** | 一次資料(https://jcgt.org/published/0009/04/01/)。Listing 1-3 に laine_karras_permutation / nested_uniform_scramble / shuffled_scrambled_sobol4d が完全掲載、suppl.zip にサンプルコード |
| cessen/sobol_burley | Burley 2020 の Rust 実装(改良ハッシュ)。seed 化・決定論を明示 |
| NVIDIA STBN SDK | STBN テクスチャと生成コード。テクスチャ型 blue-noise の作り方の参考(bartwronski/BlueNoiseGenerator にも C++ 生成器あり) |

### pt2.0-F / 将来 many-light: emissive NEE・光源サンプリング

| 参照 | 所在・要点 |
|---|---|
| **pbrt-v4** | `src/pbrt/lightsamplers.h`: `PowerLightSampler`(第一段に適切)→ `BVHLightSampler`(light BVH、円錐バウンド付き)。最も読みやすい C++ |
| **Cycles light tree** | `src/kernel/light/tree.h`(Estevez & Kulla 2018)+ 「emissive mesh を光源扱いするか」の自動判定ヒューリスティック(Blender 3.5)— 実運用で必要になる泥臭い部分まで揃う |
| **Intel OSPRay** | `modules/cpu/render/pathtracer/` の `GeometryLight` + `NextEventEstimation` — **Embree+TBB の CPU スタック上の emissive NEE/MIS** という点で umbreon に最も近いアーキテクチャ参照(ISPC 方言が難点) |

### pt2.1-A: ReSTIR-GI 空間リサンプリング

| 参照 | 所在・要点 |
|---|---|
| **DQLin/ReSTIR_PT** | GRIS(Lin 2022)の公式コード。**`Source/Falcor/Experimental/ScreenSpaceReSTIR/`** が Ouyang 2021 ReSTIR GI の正準パイプライン(initial → temporal → spatial → final、`GIReservoir.slang` 等)。Slang は C++ に近く読める。**第一の構造リファレンス** |
| **NVIDIA RTXDI** | v2.0+ が ReSTIR GI を収録(`Include/Rtxdi/GI/` の `SpatialResampling.hlsli` 等)。`Doc/RestirGI.md` / `ShaderAPI-RestirGI.md` は **biased(MIS 正規化)vs unbiased(可視性再チェック)の最も完全な文書** — 仕様書として最良。`RTXDI_CalculateJacobian` と boiling filter の実装も揃う |
| **kajiya** | `assets/shaders/rtdgi/restir_*.hlsl` + `docs/gi-overview.md`。実務チューニングの宝庫: Jacobian の距離²×cos 比を `sqrt` で減衰、M クランプ(500)、3 フレーム毎の再トレース検証。教科書的 unbiased でなく「biased をどう飼い慣らすか」の参考 |
| DoeringChristian/restirgi | Mitsuba 3 / Dr.Jit の ~1 ファイル実装。Jacobian と可視性 bias-correction を最も簡潔に書いた読本。llvm バックエンドに切り替えれば CPU でも動く教材 |
| Alegruz/Screen-Space-ReSTIR-GI | DQLin の GI 部分だけの抽出版。段階ごとの擬似コード付きで最初に読むのに向く |

注意: **C++ CPU の ReSTIR GI 公開実装は存在しない**(全て GPU シェーダ)。また
どの実装も決定論(スレッド数不変)には触れていない — §2-A の 2-pass 化は
umbreon が自分で設計する部分。なお本家 Falcor には GI パスは無い(DI 用の
RTXDIPass のみ)。

### pt2.2-D: radiance reuse / 多バウンス

| 参照 | 所在・要点 |
|---|---|
| **AMD Capsaicin (GI-1.2)** | `src/core/src/render_techniques/gi1/`(`screen_probes.hlsl`, `hash_grid_cache.hlsl`, `world_space_restir.hlsl`)。screen probe + 世界空間 hash-grid cache の 2 層構成が一式読める唯一の公開コード。論文 PDF(GI-1.0, Boissé)も公開 |
| **UE5 Lumen** | **SIGGRAPH 2022 course notes PDF(Wright et al.、advances.realtimerendering.com で公開)**が screen-probe final gather + radiance-cache feedback の設計解説の決定版。ソースも GitHub(Epic アカウント連携)で読める |
| **NVIDIA SHaRC** | 世界空間 hash-grid radiance cache のライブラリ(`include/SharcCommon.h` 等)。`docs/Integration.md` が update/resolve/query のステージングの最も明快な文書 — 世界空間フォールバックを足す場合の設計参照 |
| kajiya ircache | `assets/shaders/ircache/` + `docs/gi-overview.md` — 「bounce レイが cache 参照で終端して実質無限バウンス」というループの最良ドキュメント(voxel clipmap 型) |

注意: **Lumen 型の G-buffer 反復再利用を CPU でやった公開実装は見つからなかった**
(全部 GPU)。pt2.2 は先行実装のない設計領域であり、§2-D の
screen-space lookup + path 継続フォールバックは自前設計になる。

### pt2.3-E: GGX glossy(VNDF サンプリング)

| 参照 | 所在・要点 |
|---|---|
| **Heitz JCGT 2018** | 一次資料(https://jcgt.org/published/0007/04/01/)。Appendix A に ~15 行の `sampleGGXVNDF` 完全掲載 — 誰もが再実装している標準ルーチンで、これだけで実装できる |
| **pbrt-v4** | `src/pbrt/util/scattering.h` の `TrowbridgeReitzDistribution::Sample_wm`(Heitz 2018 VNDF)。C++ での書き方の参照 |
| Cycles | `intern/cycles/kernel/closure/bsdf_microfacet.h` の `microfacet_ggx_sample_vndf` |
| Bounded VNDF(Eto & Tokuyoshi, SIGGRAPH Asia 2023 TC) | 球冠 VNDF の数行変更で反射の分散をさらに削減(GPUOpen で PDF 公開)。Blender も採用(PR #109757)。plain VNDF が動いた後の改良 |

### §2-C 適応 spp

| 参照 | 所在・要点 |
|---|---|
| **Cycles** | `src/kernel/film/adaptive_sampling.h`: 奇数サンプル副バッファとの L1 差を輝度正規化した収束判定 + stop マスクの膨張フィルタ。**チェックポイント方式なので固定サンプル数境界で判定 = bit-exact と両立** |
| appleseed | `renderer/kernel/rendering/final/adaptivetilerenderer.cpp`: ブロック分割 + 16spp バッチのスケジューリング型 |
| LuxCoreRender | noise map でサンプラーを動的に再誘導する方式 — 決定論との両立が難しく、方式比較の参考 |

(pbrt-v4 の core には適応サンプリングは無い — 確認済み)

### 読む順番の提案

1. Cycles `pattern.h` + pbrt-v4 `lowdiscrepancy.h`(pt2.0 はこの 2 つでほぼ足りる)
2. RTXDI の Doc 2 本(仕様)→ Alegruz の抽出版 → DQLin `ScreenSpaceReSTIR/`
   (実装)→ kajiya `gi-overview.md`(チューニング)
3. Lumen course notes PDF + Capsaicin `gi1/` + SHaRC `Integration.md`(pt2.2 の設計材料)
4. Heitz 論文 + pbrt-v4 `scattering.h`(pt2.3)

### 参照リポジトリの設置

ローカルに clone して `refs/<name>/` に置く(`refs/` は .gitignore 済み)。
読むだけなので `--depth 1` の shallow clone + LFS スキップでよい
(全部で ~750MB。kajiya 327M / ReSTIR_PT 120M / RTXDI 160M が大きい)。

```sh
mkdir -p refs && cd refs
export GIT_LFS_SKIP_SMUDGE=1   # アセットは不要
git clone --depth 1 https://github.com/mmp/pbrt-v4.git
git clone --depth 1 https://github.com/blender/cycles.git
git clone --depth 1 https://github.com/DQLin/ReSTIR_PT.git
git clone --depth 1 https://github.com/Alegruz/Screen-Space-ReSTIR-GI.git
git clone --depth 1 https://github.com/EmbarkStudios/kajiya.git
git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/NVIDIA-RTX/RTXDI.git
git clone --depth 1 https://github.com/GPUOpen-LibrariesAndSDKs/Capsaicin.git
git clone --depth 1 https://github.com/NVIDIA-RTX/SHARC.git
git clone --depth 1 https://github.com/RenderKit/ospray.git
git clone --depth 1 https://github.com/DoeringChristian/restirgi.git
```

RTXDI の ReSTIR GI 実装本体は submodule(RTXDI-Library)にあるので
`--recurse-submodules` を忘れると `Libraries/Rtxdi/` が空になる。

| clone 先 | 主に見るパス | 対象段 |
|---|---|---|
| `refs/pbrt-v4/` | `src/pbrt/util/lowdiscrepancy.h`, `src/pbrt/samplers.h`, `src/pbrt/util/scattering.h`, `src/pbrt/lightsamplers.h` | pt2.0 / 2.3 / F |
| `refs/cycles/` | `src/kernel/sample/{sobol_burley,pattern}.h`, `src/kernel/film/adaptive_sampling.h`, `src/kernel/closure/bsdf_microfacet.h`, `src/kernel/light/tree.h` | pt2.0 / C / 2.3 / F |
| `refs/ReSTIR_PT/` | `Source/Falcor/Experimental/ScreenSpaceReSTIR/`(GI 正準実装)、`Source/RenderPasses/ReSTIRPTPass/Shift.slang`(GRIS shift/Jacobian) | pt2.1 |
| `refs/Screen-Space-ReSTIR-GI/` | `Rendering/ReSTIRGI/`(リポジトリ直下。抽出版、最初に読む)、`RenderPasses/ReSTIRGI/` | pt2.1 |
| `refs/kajiya/` | `docs/gi-overview.md`、`assets/shaders/rtdgi/restir_*.hlsl`、`assets/shaders/ircache/` | pt2.1 / 2.2 |
| `refs/RTXDI/` | `Doc/RestirGI.md`, `Doc/ShaderAPI-RestirGI.md`、`Libraries/Rtxdi/Include/Rtxdi/GI/{SpatialResampling,TemporalResampling,Reservoir,BoilingFilter}.hlsli`(submodule) | pt2.1 |
| `refs/Capsaicin/` | `src/core/src/render_techniques/gi1/`(screen_probes / hash_grid_cache / world_space_restir) | pt2.2 |
| `refs/SHARC/` | `docs/Integration.md`, `include/SharcCommon.h` | pt2.2 |
| `refs/ospray/` | `modules/cpu/render/pathtracer/`(GeometryLight, NextEventEstimation) | F / アーキテクチャ |
| `refs/restirgi/` | `ReSTIR-GI.md`, `ReSTIR-GI.py`(~1 ファイルの読本) | pt2.1 |

論文 PDF 類は `refs/papers/` に置く:

```sh
mkdir -p refs/papers && cd refs/papers
curl -LO https://advances.realtimerendering.com/s2022/SIGGRAPH2022-Advances-Lumen-Wright%20et%20al.pdf
curl -LO https://gpuopen.com/download/publications/GPUOpen2022_GI1_0.pdf
curl -L -o heitz2018-vndf.pdf https://jcgt.org/published/0007/04/01/paper.pdf
curl -L -o burley2020-owen-suppl.zip https://jcgt.org/published/0009/04/01/suppl.zip
curl -LO https://gpuopen.com/download/publications/Bounded_VNDF_Sampling_for_Smith-GGX_Reflections.pdf
```

補足: UE5 Lumen のソースは GitHub の Epic アカウント連携が必要なため clone リスト
からは外している(course notes PDF で足りる)。kajiya はアーカイブ済み(read-only)
だが clone は普通にできる。
