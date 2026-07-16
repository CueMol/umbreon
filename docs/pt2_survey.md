# pt2 設計サーベイ: pt1 の次に載せる現代的 GI 手法の選定

pt1(`--integrator pt1`、`--gi on` の既定)の後継として何を実装しうるかのサーベイ。
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

| 段 | 内容 | 期待効果 | コスト上限 | 規模 |
|---|---|---|---|---|
| **pt2.0** | B(サンプラー刷新)+ F 第 1 段(emissive at bounce) | denoise 後の質感改善 + 発光表現 | ×1.0 | 小 |
| **pt2.1** | A(ReSTIR-GI spatial、biased 既定 + unbiased フラグ) | 実効 spp ×8-20、品質の主ジャンプ | ×1.5(biased) | 中 |
| **pt2.2** | D(screen-space 多バウンス、bounces=∞ 既定) | 深部ポケットの正しい明るさ | ×1.2 | 中 |
| **pt2.3** | E(glossy 間接光、フェイク reflection の置換) | 映り込み・wet look | glossy 面比例 | 中 |
| **pt3** | G(temporal / RenderSession API) | アニメの高速化 + 安定化 | 別トラック | 大 |

順序の根拠: B は全候補と直交し全段の土台になるので最初。A は品質目標の本丸で、
D と E は A の reservoir/バッファ構造に乗ると実装が薄くなる(D の lookup 対象も
E_spec も「gather グリッド上の完成バッファ」であり、A で整備するインフラと同じ)。

## 5. go/no-go 判定基準

各段とも `1ab0_scene1` 1200²(+ 凹部の多い `1ab0_scene6_densurf` 系)で計測:

- **品質段(pt2.0/2.1)**: 等時間比較で ultra 参照に対する PSNR が pt1 draft 比 +2dB
  以上、または等 PSNR で時間 30% 減。加えて denoise 後の目視比較
  (ユーザー判定 — 自己判定はしない)
- **多バウンス段(pt2.2)**: bounces=∞ の結果が traced bounces=8 と PSNR 40dB 以上で
  一致し、コストが bounces=2 比 +20% 以内
- **表現段(pt2.3/F)**: 該当マテリアルなしのシーンで bit-exact(完全 skip の証明)、
  ありのシーンで目視判定
- 全段共通: `refactor_check.sh` の既存ケースが byte-identical(pt2 は opt-in なので
  既定経路に影響しないこと)、スレッド数を変えて bit-exact、`task test` 全通過
