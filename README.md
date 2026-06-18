# jomon_period

縄文土器の PLY メッシュから縄目圧痕の**繰り返し周期（ピッチ・方向）を全自動推定**する C++17 CLI。

## クイックスタート

### 1. vcpkg のセットアップ（初回のみ）

```powershell
# Windows
scripts\bootstrap_vcpkg.ps1
```

```bash
# macOS / Linux
bash scripts/bootstrap_vcpkg.sh
```

### 2. ビルド

```powershell
# Windows (MSVC – ビューアなし)
cmake --preset windows-msvc-release
cmake --build build\windows-msvc-release --config Release

# Polyscope ビューア付き（初回ビルドは Polyscope の取得・コンパイルが追加で発生）
cmake --preset windows-msvc-viewer
cmake --build build\windows-msvc-viewer --config Release
```

```bash
# macOS
cmake --preset macos-clang-release
cmake --build build/macos-clang-release
```

### 3. 実行

```bash
# 周期推定 CLI（最終ゴール）
jomon_period input.ply

# オプション例
jomon_period input.ply --verbose --grid-pitch 0.15 --period-min 2.0 --period-max 5.0

# 3D ビューア（JOMON_BUILD_VIEWER=ON でビルドした場合）
jomon_viewer input.ply
jomon_viewer input.ply --highlight-target  # 対象面を緑でハイライト
```

### 出力例

```
pitch=3.214mm  direction=42.5deg  dxdy=(2.371, 2.172)mm  confidence=0.87
grid=0.20mm  detrend_window=12
```

## テスト

```bash
ctest --preset windows-msvc-release  # Windows
ctest --preset macos-clang-release   # macOS
```

## パイプライン

1. **PLY 読み込み** (happly) → Eigen 行列
2. **対象面切り出し** 法線クラスタリング + 最大連結成分
3. **高さマップ化** PCA 平面フィット → 格子サンプリング
4. **detrend** 移動平均差分（低周波うねり除去）
5. **2D 自己相関** マスク付き FFT 法 → 周期ベクトル推定

## CLI オプション

| オプション | 既定値 | 説明 |
|-----------|--------|------|
| `--grid-pitch mm` | 0.20 | 高さマップの格子サイズ (mm) |
| `--detrend-window n` | 12 | detrend 箱平均の半幅 (格子数) |
| `--period-min mm` | 1.5 | 探索する最小ピッチ (mm) |
| `--period-max mm` | 6.0 | 探索する最大ピッチ (mm) |
| `--normal-angle deg` | 30.0 | 対象面法線クラスタリング角度閾値 |
| `--verbose` | – | 進捗を stderr に出力 |

## 依存ライブラリ

| ライブラリ | 用途 | 導入方法 |
|-----------|------|---------|
| [happly](https://github.com/nmwsharp/happly) | PLY 読み込み | FetchContent (自動) |
| Eigen | 線形代数・FFT | vcpkg |
| fmt | CLI 出力整形 | vcpkg |
| [Polyscope](https://polyscope.run/) (任意) | 3D ビューア | FetchContent (`JOMON_BUILD_VIEWER=ON`) |
