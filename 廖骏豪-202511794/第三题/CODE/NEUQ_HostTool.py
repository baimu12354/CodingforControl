#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
第三题 串口调参与电机闭环控制 —— PC 端测试上位机（没有 VOFA+ 时用）

对应固件：CODE/NEUQ_Config.h
  · 下行帧（两种帧头等价，都不带校验）：
        AA 55 CMD LEN DATA[LEN]
        CC DD CMD LEN DATA[LEN]
  · 上行帧（FireWater 文本，7 通道，50Hz）：
        目标,实际,Kp,Ki,Kd,输出,使能\\r\\n
  · 串口：USART1(PA9/PA10) 115200 8N1，接 ST-Link 虚拟串口

依赖：
    pip install pyserial
    pip install matplotlib        # 只有画图时才需要

用法示例：
    python NEUQ_HostTool.py -p COM9 monitor                 # 只看数据（默认 10s）
    python NEUQ_HostTool.py -p COM9 monitor --duration 30 --plot
    python NEUQ_HostTool.py -p COM9 target 100              # 目标 100 RPM
    python NEUQ_HostTool.py -p COM9 start                   # 启动
    python NEUQ_HostTool.py -p COM9 stop                    # 停止
    python NEUQ_HostTool.py -p COM9 pid 20 150 0            # PID
    python NEUQ_HostTool.py -p COM9 openloop 2000           # 开环调试
    python NEUQ_HostTool.py -p COM9 demo                    # 自动跑一遍验收流程
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺少 pyserial，请先执行:  pip install pyserial")

# ------------------------------------------------------------------ 协议常量
HEAD0, HEAD1 = 0xAA, 0x55
LITE0, LITE1 = 0xCC, 0xDD          # VOFA+ 滑条常用的轻量帧头，固件同样接受

CMD_SET_PID, CMD_SET_TARGET, CMD_START, CMD_STOP = 0x01, 0x02, 0x03, 0x04
CMD_SET_GAIN2, CMD_CLEAR, CMD_QUERY, CMD_OPENLOOP = 0x05, 0x06, 0x07, 0x08
CMD_SET_KP_ONE, CMD_SET_KI_ONE, CMD_SET_KD_ONE = 0x11, 0x12, 0x13
CMD_FEEDBACK = 0x81

CH_NAMES = ["目标", "实际", "Kp", "Ki", "Kd", "输出", "使能"]


def f32(*values) -> bytes:
    """IEEE754 单精度，小端"""
    return struct.pack("<%df" % len(values), *values)


def build_frame(cmd: int, payload: bytes = b"", lite: bool = True) -> bytes:
    """组一帧：<头> CMD LEN DATA （无校验字节）"""
    h = (LITE0, LITE1) if lite else (HEAD0, HEAD1)
    return bytes([h[0], h[1], cmd, len(payload)]) + payload


# ------------------------------------------------------------------ 串口封装
class Board:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.buf = bytearray()
        self.latest = None
        self.cmd_count = 0
        self.frame_count = 0
        self.bad_count = 0

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, cmd, payload=b""):
        self.ser.write(build_frame(cmd, payload))
        self.cmd_count += 1

    # ---- 具体命令 ----
    def set_pid(self, kp, ki, kd):  self.send(CMD_SET_PID, f32(kp, ki, kd))
    def set_target(self, rpm):      self.send(CMD_SET_TARGET, f32(rpm))
    def start(self):                self.send(CMD_START)
    def stop(self):                 self.send(CMD_STOP)
    def query(self):                self.send(CMD_QUERY)
    def clear(self):                self.send(CMD_CLEAR)
    def openloop(self, duty):       self.send(CMD_OPENLOOP, f32(duty))

    # ---- 解析：FireWater 文本，一帧一行 ----
    def poll(self):
        data = self.ser.read(4096)
        if data:
            self.buf.extend(data)

        out = []
        while True:
            i = self.buf.find(b"\n")
            if i < 0:
                break
            line = bytes(self.buf[:i]).strip(b"\r")
            del self.buf[:i + 1]
            if not line:
                continue
            try:
                vals = [float(x) for x in line.decode("ascii", "strict").split(",")]
            except (ValueError, UnicodeDecodeError):
                self.bad_count += 1
                continue
            if len(vals) < 2:
                self.bad_count += 1
                continue
            self.latest = vals
            self.frame_count += 1
            out.append(vals)

        # 缓冲别无限增长（比如波特率设错了收到一堆乱码）
        if len(self.buf) > 4096:
            self.buf.clear()
        return out


def fmt(vals):
    return "  ".join("%s=%.2f" % (CH_NAMES[i] if i < len(CH_NAMES) else "ch%d" % i, v)
                     for i, v in enumerate(vals))


# ------------------------------------------------------------------ 子命令
def cmd_monitor(b, args):
    print("开始接收（Ctrl+C 退出）…… 目标/实际曲线也可以用 VOFA+ 的 FireWater 直接看")
    t0 = time.time()
    rows = []
    try:
        while time.time() - t0 < args.duration:
            for v in b.poll():
                rows.append((time.time() - t0, v))
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass

    if b.bad_count:
        print("注意：有 %d 行解析失败（检查波特率是否 115200 / 数据格式是否 FireWater）" % b.bad_count)
    print("收到 %d 帧，最后一条: %s" % (b.frame_count, fmt(b.latest) if b.latest else "无"))

    if args.save and rows:
        with open(args.save, "w", encoding="utf-8") as f:
            f.write("t," + ",".join("ch%d" % i for i in range(len(rows[0][1]))) + "\n")
            for t, v in rows:
                f.write("%.4f," % t + ",".join("%.4f" % x for x in v) + "\n")
        print("已保存 %d 行到 %s" % (len(rows), args.save))

    if args.plot and rows:
        try:
            import matplotlib.pyplot as plt
            ts = [r[0] for r in rows]
            plt.plot(ts, [v[0] for _, v in rows], label="target")
            plt.plot(ts, [v[1] for _, v in rows], label="actual")
            plt.xlabel("t / s")
            plt.ylabel("speed / RPM")
            plt.grid(True)
            plt.legend()
            plt.show()
        except ImportError:
            print("未安装 matplotlib，跳过画图（pip install matplotlib）")


def cmd_demo(b, args):
    def hold(sec):
        end = time.time() + sec
        while time.time() < end:
            b.poll()
            time.sleep(0.005)

    print("[0] 开环 +2000 确认接线与测速方向（3s）")
    b.openloop(2000)
    hold(3)
    print("    当前: %s" % fmt(b.latest) if b.latest else "    没收到回传")
    print("    若“实际”为负 -> 改 NEUQ_ENCODER_DIR_SIGN；电机转向反了 -> 改 NEUQ_MOTOR_DIR_SIGN（只改一个）")
    b.openloop(0)
    hold(0.5)

    print("[1] 下发 PID %g/%g/%g" % (args.kp, args.ki, args.kd))
    b.set_pid(args.kp, args.ki, args.kd)
    hold(0.3)

    print("[2] 目标 %g RPM -> 启动" % args.target)
    b.set_target(args.target)
    b.start()
    hold(6)
    print("    当前: %s" % (fmt(b.latest) if b.latest else "无"))

    print("[3] 升速到 %g RPM" % min(args.target * 1.5, 200.0))
    b.set_target(min(args.target * 1.5, 200.0))
    hold(6)
    print("    当前: %s" % (fmt(b.latest) if b.latest else "无"))

    print("[4] 降速回 %g RPM" % args.target)
    b.set_target(args.target)
    hold(6)
    print("    当前: %s" % (fmt(b.latest) if b.latest else "无"))

    print("[5] 请用手给轮子加负载并松开（5s）")
    hold(5)
    print("    当前: %s" % (fmt(b.latest) if b.latest else "无"))

    print("[6] 连续运行 30s，期间反复改目标、重发参数（曲线不得因通信中断）")
    end = time.time() + 30
    i = 0
    while time.time() < end:
        i += 1
        b.set_target(args.target if i % 2 else min(args.target * 1.3, 200.0))
        if i % 10 == 0:
            b.set_pid(args.kp, args.ki, args.kd)
            b.query()
        hold(1.0)
        print("    t=%2d  %s" % (i, fmt(b.latest) if b.latest else "无"))

    print("[7] 停止")
    b.stop()
    hold(2)
    print("    当前: %s" % (fmt(b.latest) if b.latest else "无"))
    print("完成：共发 %d 条命令，收到 %d 帧" % (b.cmd_count, b.frame_count))


def main():
    ap = argparse.ArgumentParser(description="第三题上位机测试工具（FireWater 7 通道）")
    ap.add_argument("-p", "--port", required=True, help="串口号，如 COM9 或 /dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="波特率（默认 115200）")
    sub = ap.add_subparsers(dest="action", required=True)

    sub.add_parser("start", help="启动闭环")
    sub.add_parser("stop", help="停止")
    sub.add_parser("query", help="立即回传一帧")
    sub.add_parser("clear", help="清 PID 积分/内部状态")

    p = sub.add_parser("pid", help="下发 PID 参数")
    p.add_argument("kp", type=float)
    p.add_argument("ki", type=float)
    p.add_argument("kd", type=float)

    p = sub.add_parser("target", help="下发目标转速(RPM)")
    p.add_argument("rpm", type=float)

    p = sub.add_parser("openloop", help="开环调试占空比(-10000~10000)")
    p.add_argument("duty", type=float)

    p = sub.add_parser("gain2", help="前馈增益 + 输出变化率限幅")
    p.add_argument("kff", type=float)
    p.add_argument("slew", type=float)

    p = sub.add_parser("monitor", help="持续接收并显示")
    p.add_argument("--duration", type=float, default=10.0, help="接收时长(s)")
    p.add_argument("--save", default=None, help="保存 CSV 路径")
    p.add_argument("--plot", action="store_true", help="结束后用 matplotlib 画图")

    p = sub.add_parser("demo", help="自动跑一遍验收流程")
    p.add_argument("--kp", type=float, default=20.0)
    p.add_argument("--ki", type=float, default=150.0)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--target", type=float, default=100.0, help="基准目标转速(RPM)")

    args = ap.parse_args()
    b = Board(args.port, args.baud)
    try:
        a = args.action
        if   a == "start":    b.start();    print("已发送 启动")
        elif a == "stop":     b.stop();     print("已发送 停止")
        elif a == "query":    b.query();    print("已发送 查询")
        elif a == "clear":    b.clear();    print("已发送 清 PID")
        elif a == "pid":      b.set_pid(args.kp, args.ki, args.kd); print("已发送 PID %g %g %g" % (args.kp, args.ki, args.kd))
        elif a == "target":   b.set_target(args.rpm); print("已发送 目标 %g RPM" % args.rpm)
        elif a == "openloop": b.openloop(args.duty);  print("已发送 开环占空比 %g" % args.duty)
        elif a == "gain2":    b.send(CMD_SET_GAIN2, f32(args.kff, args.slew)); print("已发送 kff/slew %g %g" % (args.kff, args.slew))
        elif a == "monitor":  cmd_monitor(b, args); return
        elif a == "demo":     cmd_demo(b, args);    return

        # 普通命令：等一会儿，把生效后的回传值打出来
        end = time.time() + 0.5
        while time.time() < end:
            b.poll()
            time.sleep(0.005)
        print("回传: %s" % (fmt(b.latest) if b.latest else "（没收到回传：检查波特率/接线/数据格式）"))
    finally:
        b.close()


if __name__ == "__main__":
    main()
