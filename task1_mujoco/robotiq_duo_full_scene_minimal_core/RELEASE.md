# 发布与开发指南 (Release & Development Guide)

交付物与对应用户：

| 交付物 | 目标用户 | 说明 |
|---|---|---|
| `build/duo_teleop_<ver>.zip`（**明文源码包**，默认） | Windows / Linux 原生用户，含 VR | 挂到 GitHub Release 下载 |
| 同上但 `--obfuscate`（PyArmor 混淆，可选） | 需要源码保密时 | 需 pyarmor，正式分发建议购 license |
| `duo-teleop-runtime` Docker 镜像（release/compose.yaml） | 跨平台一致性测试（键盘/手柄，无 ROS） | `docker-run.sh` 封装 |
| `duo-teleop-eval` Docker 镜像（release/compose.yaml） | Ubuntu / WSL2 跑 ManipulationNet 评测 / GELLO（都需要 ROS 2） | `eval.sh` 封装 |
| 本仓库 | 开发者 | https://github.com/2houyuhang/EBiM_Benchmark_task1 |

---

## 用户指南

### A. conda 源码包（键盘 / 手柄 / VR，win + linux 通用）

一键启动（唯一前置条件：装好 miniconda）——首次运行自动创建环境：

```bash
unzip duo_teleop_<ver>.zip && cd duo_teleop_<ver>
python start.py                    # 键盘（首次自动建环境，几分钟）
python start.py --input gamepad    # 手柄
python start.py --input vr         # VR（见下）
python start.py --no-viewer        # 自检
```

（等价的手动方式：`conda env create -f environment.yml && conda activate
duo-teleop && python main.py ...`）

VR 运行时（代码是纯 OpenXR，装好任一运行时即可）：
- Windows + Quest：Meta Quest Link，设为活动 OpenXR 运行时
- Ubuntu + Quest：WiVRn（推荐，无需 Steam）或 ALVR+SteamVR；需 X11 会话
- Index/Vive：SteamVR

### B. Docker 评测栈（Ubuntu / WSL2，含 ROS 2 + mnet client）

仓库根目录的 `eval.sh` 是一键封装（等价的裸 compose 命令见文件内注释）：

```bash
# 0. 一次性：编辑 mnet_client-ros_2/config/team_config.json
#    camera_image_topic=/mujoco/camera/image_raw, autonomy_level=0, file_dir=/ws/out
./eval.sh sim        # 首次自动构建镜像，然后弹出 sim 窗口（WSL2 免配置；原生 Linux 先 xhost +local:docker）
./eval.sh client     # 第二个终端：交互式 client，输入 cable_management
#    -> 出 one-time code 后，在 sim 终端输入: code <TEXT>
#    -> 非 Tier2 自动跳过；Tier2 fixture 自动随机化；完成后在 sim 窗口按 F
```

评测流程细节见 README.md 的 "ManipulationNet eval" 一节。

---

## 开发者工作流（你）

日常开发**完全不变**：改 `teleop/`，用 `python main.py --no-viewer` 冒烟，正常 commit。
发布文件（environment.yml / release/*）引用代码而不复制代码，只有两种情况要动它们：

- 新增/升级了 python 依赖 → 在 `environment.yml` 和 `release/Dockerfile.eval` 各加一行
- 改了评测编排（topic 名、启动命令）→ `release/compose.yaml`

Docker 内调试最新代码不用重建镜像：取消 compose.yaml 里 DEV MODE 那行挂载的注释。

## 发布 Docker 镜像（组织者要求的交付形态：用户拿镜像直接跑）

镜像 `duo-teleop-eval`（约 3.65GB，含 sim 全部五种输入模式 + ROS 2 Humble +
官方 mnet client + teleop 发布包）就是"一切都在里面"的交付物。用户侧只需要
一个 `release/compose.dist.yaml`（改名 compose.yaml 放进任意空文件夹）：

```bash
docker compose pull && docker compose up sim        # 仿真+评测桥
docker compose run --rm client                      # 第二终端: mnet client
docker compose run --rm shell python3 main.py --no-viewer   # 任意其他用法
```

发布到 GitHub Container Registry（推荐，无体积上限，跟仓库同账号管理）：

```bash
# 一次性: 生成 PAT(classic, 勾 write:packages) 后登录
echo <PAT> | docker login ghcr.io -u 2houyuhang --password-stdin
# 每次发版:
docker compose -f release/compose.yaml build sim
docker tag duo-teleop-eval:latest ghcr.io/2houyuhang/ebim-task1:latest
docker tag duo-teleop-eval:latest ghcr.io/2houyuhang/ebim-task1:v0.1
docker push ghcr.io/2houyuhang/ebim-task1:latest
docker push ghcr.io/2houyuhang/ebim-task1:v0.1
# 镜像默认 private; 要让比赛用户能 pull: github.com -> 你的头像 -> Packages
# -> ebim-task1 -> Package settings -> Change visibility (public)，或给指定
# 用户/组织授 read 权限
```

把 `compose.dist.yaml` 作为附件挂到 GitHub Release（或直接发给用户），用户
无需 clone 任何源码。不能容器化的部分照旧：手柄需原生 Linux `/dev/input`，
VR 需原生运行（start.bat / start.sh）。

## 重建发布版并发到 GitHub Releases（每次发版）

```bash
# 1. 打版本标签并推送
git commit ... && git tag v0.1 && git push && git push origin v0.1
# 2. 一键构建: 冒烟门禁 -> 源码 zip -> docker 镜像（docker 缺失自动跳过）
python release/publish.py
# 3. 发布到 GitHub Release（gh CLI，一条命令；首次先 gh auth login）
gh release create v0.1 build/duo_teleop_v0.1.zip \
  --title "v0.1" --notes "keyboard/gamepad/VR teleop + mnet eval"
#    或网页操作: 仓库页 -> Releases -> Draft a new release -> 选 tag v0.1
#    -> 把 build/duo_teleop_v0.1.zip 拖进 assets -> Publish
```

可选项：
- `python release/publish.py --obfuscate [--cross]`：改出 PyArmor 混淆包
  （需 `pip install pyarmor`，`--cross` 另需 `pyarmor.cli.runtime`；混淆不加
  `--cross` 时产物只能在构建平台同类系统上运行；对外分发混淆包建议购买
  PyArmor license）
- 镜像分发：`docker tag duo-teleop-eval:latest <registry>/duo-teleop-eval:v0.1
  && docker push ...`（GitHub 可用 ghcr.io）
- 单独步骤：`python release/build_release.py [--obfuscate]`（只出 zip）、
  `docker compose -f release/compose.yaml build`（只建镜像）

注意：
- Release asset 单文件上限 2GB，我们的 zip（~65MB）远低于限制。
- XML 场景和 mesh 资产是数据文件，混淆也无法保密。
- 房间场景 mesh/贴图提取自第三方模型（3d66），**公开发布前需确认再分发许可**。
