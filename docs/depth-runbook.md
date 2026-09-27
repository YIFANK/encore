# 深度相机 runbook(结论:可行,但**必须以 root 运行**)

> **2026-07-31 更正。** 本文档开头原先写着"本机不可行,方案作废",依据是 14:57 那次
> 普通用户身份的枚举失败。**这个结论是错的**,同一天更晚的测试推翻了它——当时写下
> "root 也改变不了"时,root 其实还没试过。机器上留下的证据:
>
> | 日志 | 身份 | 结果 |
> |---|---|---|
> | `depth_diag.log` 14:57 | 普通用户 | enumeration FAILED — failed to set power state |
> | `depth_clean_test.log` 14:57 `--- as root ---` | root | 4 台 D405 全部枚举成功 |
> | `depth_stream_test.log` 15:03 | root | **存下了真实的 `rs-save-to-disk-output-Depth.png`** |
>
> 所以:**librealsense 在这台 Mac 上能拿到 D405 的深度,条件是进程以 root 跑在 GUI
> 上下文里**(SSH 上下文永远不行——那是 macOS 按启动上下文发相机权限,与 root 无关,
> 两个条件都要满足)。实测 1–2 台稳定,4 台不稳定。
>
> 这条更正有实际后果:**没有深度,SAM2 掩码在 `deproject_region` 里根本走不到**
> ——无深度分支在函数开头就返回了。真机要用掩码,得先按本文档把深度打开。
>
> **重要副作用**:USB 拔插后长期运行的 camera_dashboard 会失去相机(它的会话失效),
> 必须在 GUI 终端里 Ctrl-C 重启 `./start_camera_dashboard.command` 才能恢复画面。

# 旁挂 depth server 现场验证

**背景**:macOS 按启动上下文发相机权限,SSH 进程永远打不开 D405
(`failed to set power state`,已于 2026-07-30 实测确认,包括相机空闲时)。
解法:在 **GUI 上下文**跑一个旁挂 `tools/depth_server.py`,独占**一台** D405,
通过 `http://127.0.0.1:8766` 向 Heron 提供对齐的 RGB-D。dashboard 与其余 3 台相机不受影响。

**两个已知代价**(设计内,不是故障):
1. RealSense 设备独占——被 depth server 接管的那台相机会从 dashboard 消失;
2. 该设备从 AVFoundation 消失后,**其余相机的 index 可能重排**
   (现映射 0=左腕, 1=high, 2=low, 3=右腕)。若 dashboard 画面串位,
   改 dashboard 里的 index 映射即可(或喊 Claude 远程改)。

## 先决条件:清干净相机占用(2026-07-30 实测踩到的坑)

librealsense 在 macOS 上走**原始 USB**,只要有任何进程通过 AVFoundation 打开着这些
相机,它就会报 `failed to set power state`;反复失败后设备会卡死到**连枚举都返回 0**
(GUI 里表现为 `No device connected`)。所以下面每一步之前都要:

1. **退出 Chrome 里占相机的页面**(实测发现一个 Chrome Helper 挂了 4 天占着相机)。
   最省事:`Cmd+Q` 完全退出 Chrome,做完再开。
2. `curl -X POST http://127.0.0.1:8765/api/cameras/stop` 释放 dashboard。
3. 若已经卡死(枚举=0),**拔插 USB hub 复位**:四台 D405 都挂在同一个 USB3.1 hub 上,
   把 hub 到 Mac 的那根线拔掉 5 秒再插回,四台一起复位。复位后**立刻**跑验证,
   别让 dashboard/Chrome 先抢走。

注意:AVFoundation 里这些设备名字叫 "…405 **Depth**",但那只是 Intel 的产品名,
该通道给的是**彩色图**(已用 dashboard 快照证实),深度拿不到——只能走 librealsense。

## 为什么四台开不起来(2026-07-31,**原因未定**)

先说不成立的结论。本文档一度断言这是总线供电不足并称已坐实,**那是错的**:
USB 3.0 规范里自供电 hub 每口给 900 mA、总线供电 hub 每口只给 100 mA,而
`system_profiler` 在每台 D405 上报的是 `Current Available: 900`——这恰恰说明
hub 是**自供电**的。单口预算够(900 供 vs 720 需),不是"差 3.2 倍"。

已知的硬事实:

```
Intel RealSense D405   Current Required  (mA): 720   ← 每台流式取电
                       Current Available (mA): 900   ← 所在口的预算
拓扑:Mac 一个口 → USB3.2 Hub → USB3.1 Hub → 4 × D405(两级串联)
```

现象:四台**全部正常枚举**,第一台 open 成功,其余报 `No device connected`。
只留一台时立刻成功。连开两台成功过,但那是 `identify_cameras.py` 在两台之间
`stop()` 并等 1.5 秒的结果——**始终串行,从未真正并发**。

还站得住的两个候选解释,都没排除:

1. **hub 电源的总输出**。单口 900 是 hub 的宣称值,适配器实际能不能持续供
   4 × 720 = 2.88 A 是另一回事;不少"带电源"hub 配的是 2 A 适配器。软件侧读
   不到实际输出,要用换 hub 或电流表来判。
2. **macOS 上的 librealsense 多设备路径**。它走原始 USB,是最没被测试过的组合
   ——连枚举都要 root(Linux 不需要)。同样能解释"枚举成功、open 失败"。

判别方法:把四台分到**两个不同的 Mac 主机口**(不同控制器)各两台。如果能同时
开两台,偏向电源/带宽;如果还是只有一台能开,偏向 librealsense。

想跑满四台,按代价从低到高:

1. **换自供电 USB hub**(带电源适配器,非总线供电)。要求总输出 ≥ 4 A 且
   **单口能给到 900 mA**——不少廉价「带电源」hub 总功率够但单口限流,没用。
   同时**拆掉串联**,一级直连。验收:`system_profiler` 里每台 D405 的
   `Current Available` ≥ 900,且 `identify_cameras.py` 跑出 4/4。
2. **别让四台都出深度。** 只有 cam_high(顶多加 cam_low)需要深度,腕部相机
   给 VQA 看 RGB 就够。`depth_server --color-only` 就是干这个的。这是不改硬件
   今天就能用的配置。
3. **把相机挪到 Linux 主机**,`depth_server` 本来就是网络服务,改配置里的 URL
   即可,不动架构。除了供电,这还一并解决两个 macOS 特有问题:librealsense 在
   macOS 上走原始 USB、多设备是最没被测试的路径(连枚举都要 root,Linux 不用);
   以及四台共享一条 5 Gb/s 上行,分辨率一高带宽就紧张,换 hub 解决不了,需要
   多个主机控制器——台式机才有。

## 现场步骤(预计 5–10 分钟,需在 Mac 前的 Terminal.app 里操作)

```bash
# 0) 只有 camera_dashboard 在跑时才需要释放相机。先确认它在不在:
pgrep -fl camera_dashboard || echo "没在跑 —— 跳过这步"
# 2026-07-31 实测:它没在跑,而 8765 已经被 claude-chat-server 占用,
# 下面这条旧命令因此返回 404。用 pgrep 判断,别照抄端口。
curl -X POST http://127.0.0.1:8765/api/cameras/stop   # 仅当 dashboard 确实在跑

# 1) 验证 GUI 上下文里 pyrealsense2 真能出深度(整个方案的前提)。
#    诊断脚本把结果写进 ~/Heron/depth_diag.log,Claude 可远程读取:
sudo ~/Heron/.venv/bin/python ~/Heron/tools/depth_diag.py
#    → 末行 "RESULT: DEPTH OK ..." = 通过。
#    → 首次可能弹相机授权框,点允许后重跑。
#    → 不加 sudo 必定 failed to set power state,那不是故障,是这台机器的常态。
#    → 加了 sudo 仍然 device count = 0 → 回到上面"先决条件"三步(退 Chrome、
#      停 dashboard 相机、拔插 hub),再重试一次;仍失败则转 Linux 盒子方案。
#    (老办法:.venv/bin/python .venv/bin/python-tutorial-1-depth.py,出现随距离变化的
#     字符画即通过;开头那句 SyntaxWarning 是示例脚本的老写法,与成败无关。)

# 2) 找 overhead 相机的序列号。**2026-07-31 已经做过,答案是 218622271652**,
#    已写进 configs/abaka.yaml,正常情况下不必重做。
#
#    重做时用 tools/identify_cameras.py,不要用 depth_server --auto:
#      sudo ~/Heron/.venv/bin/python ~/Heron/tools/identify_cameras.py
#    --auto 会四台一起开,而这台机器做不到——上一次尝试四台在几秒内全部
#    "No device connected"。实测规律:第一台能开,后面的一律失败,即使它们
#    都正常枚举出来了。原因未定,见上面「为什么四台开不起来」。
#
#    排查时先确认不是被占用:实测 VDCAssistant 的 USB/video 句柄数为 0,
#    camera_dashboard 没在跑。**设备先正常枚举、open 时才报 No device
#    connected**,这不是被占用的表现——被占用的设备根本不会先出现。
#
#    (旧办法,每台拍一张肉眼认:)
cd ~/Heron && .venv/bin/python - <<'EOF'
import pyrealsense2 as rs, numpy as np
from PIL import Image
ctx = rs.context()
serials = [d.get_info(rs.camera_info.serial_number) for d in ctx.query_devices()]
print("devices:", serials)
for s in serials:
    pipe = rs.pipeline(ctx); cfg = rs.config()
    cfg.enable_device(s); cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
    try:
        pipe.start(cfg)
        for _ in range(10):
            frames = pipe.wait_for_frames(5000)  # let auto-exposure settle
        Image.fromarray(np.asanyarray(frames.get_color_frame().get_data())).save(f"/tmp/cam_{s}.png")
        pipe.stop(); print(s, f"-> /tmp/cam_{s}.png")
    except Exception as e:
        print(s, "FAILED:", type(e).__name__, e)
EOF
open /tmp/cam_*.png   # 看哪张是顶视图,记下它的序列号

# 3) 正式启动(留在一个 Terminal 标签里跑着)。
#    sudo 是必须的:普通用户身份枚举会 failed to set power state(见文档开头的更正)。
#    参数是 --camera ROLE=SERIAL,没有 --serial 这个开关。
sudo .venv/bin/python tools/depth_server.py --camera cam_high=218622271652 --port 8766
#    序列号已经写进 configs/abaka.yaml,所以下面这条等价:
sudo .venv/bin/python tools/depth_server.py --config configs/abaka.yaml --port 8766

# 3b) 收尾时不要只按 Ctrl-C。librealsense 是从 macOS 的 UVC 驱动手里抢走相机的,
#     直接退出不会还回去,AVFoundation 那条路就断了(其余相机的索引也会乱)。
sudo .venv/bin/python tools/depth_server.py --handback

# 4) 其余相机走 UVC,按索引寻址,而索引在 cam_high 被 librealsense 拿走之后
#    必然重排。别去回忆插的顺序,直接重认(不需要 sudo):
.venv/bin/python tools/identify_uvc.py     # 图存在 /tmp/uvcid/,照图改配置里的 index

# 5) 验收(这步在任何 ssh 会话里都行):
curl -s http://127.0.0.1:8766/health
```

验收通过后,SSH 侧的一切(标定、Heron 运行)都不再需要人到现场。

## 之后(可远程完成)

- `configs/abaka.yaml` 里取消 `cam_high.depth_server` 的注释;
- 有了深度,cam_high 标定改用完整外参模式(不带 `--planar`):
  `heron calibrate --config configs/abaka.yaml --camera cam_high --arm right`
- `heron doctor --config configs/abaka.yaml` 会显示 depth server 健康状态。

## 常驻化(可选,推荐)

把 server 做成登录自启的 LaunchAgent(仍属 Aqua 会话,权限保留):

```bash
cat > ~/Library/LaunchAgents/ai.heron.depthserver.plist <<'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>Label</key><string>ai.heron.depthserver</string>
  <key>ProgramArguments</key><array>
    <string>/Users/abaka/Heron/.venv/bin/python</string>
    <string>/Users/abaka/Heron/tools/depth_server.py</string>
    <string>--camera</string><string>cam_high=218622271652</string>
    <string>--port</string><string>8766</string>
  </array>
  <key>RunAtLoad</key><true/>
  <key>KeepAlive</key><true/>
  <key>StandardOutPath</key><string>/Users/abaka/Heron/depth_server.log</string>
  <key>StandardErrorPath</key><string>/Users/abaka/Heron/depth_server.log</string>
</dict></plist>
EOF
launchctl load ~/Library/LaunchAgents/ai.heron.depthserver.plist
```

首次以 LaunchAgent 身份跑可能再弹一次授权框(需在机器前点一次允许)。

## 顺手强烈建议

在 系统设置 → 通用 → 共享 里打开 **屏幕共享(Screen Sharing)**。
以后所有需要 GUI 的操作(授权框、重启 dashboard)都能通过 Tailscale 远程完成,
不用再跑实验室。
