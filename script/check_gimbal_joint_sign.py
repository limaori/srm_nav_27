#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核对 gimbal_joint_sign（云台关节角符号）—— 首次上车必做。

`start_real_nav.sh` 启动时会提示:

    首次上车核对: 云台不动、手转底盘, RViz 里 base_link 要跟着转;
    反了改 srm27_nav_protocol.yaml 的 gimbal_joint_sign (+1.0/-1.0)

本脚本把"用眼睛看 RViz"换成一条可判定的数值判据, 不需要盯着屏幕猜方向。

================================ 为什�� base_link 会转 ================================

TF 链是::

    odom --(LIO 发布)--> livox_imu --(URDF 固定)--> gimbal_yaw --(gimbal_yaw_joint)--> base_link

`odom -> base_link` 由 `sensor_scan_generation` **反推**得到:

    odom->base_link = odom->livox_imu * (base_link->livox_imu)^-1

云台锁住、手转底盘时: 雷达没动 -> `odom->livox_imu` 不变; 关节角 θ 变了 -> 反推让
`base_link` 跟着**手**转。所以:

  * `base_link`（红标 chassis_mark）跟着手转  <- 判据
  * `livox_imu` / `gimbal_yaw`（蓝标 gimbal_mark）在 RViz 里**不动**
  * `base_link -> gimbal_yaw` 的 yaw **就是** θ = URDF 关节角

方向定义（俯视逆时针为正, 与全仓库一致）:

    云台不动, 手把底盘逆时针转 φ  =>
        下位机 /chassis_to_gimbal_yaw_deg = +φ       （底盘相对云台）
        θ = base_link -> gimbal_yaw 的 yaw = -φ      （云台相对底盘, **必须是负的**）
        odom -> base_link 的 yaw  = +φ               （和手同向, 大小相同）

θ 的正负就是 `gimbal_joint_sign` 的直接体现: θ = gimbal_joint_sign × deg × π/180,
默认 -1.0 时 θ = -φ。**手转逆时针而 θ 变正, 就是符号反了。**

================================ 用法 ================================

    # 车上已在跑导航栈（LIO + srm27_nav_protocol + relay + robot_state_publisher）
    ./script/check_gimbal_joint_sign.py --hand ccw

    # 你实际是顺时针转的底盘
    ./script/check_gimbal_joint_sign.py --hand cw

    # 带命名空间 / 换采样时长
    ./script/check_gimbal_joint_sign.py --ns /red_standard_robot1 --seconds 15

采样窗口内请**慢速**把底盘相对云台转 30~45°, 再转回来（转回来是为了顺便看
`atan2` 归一化在 ±180° 附近不出毛病; 幅度小则会低于判定门槛）。

================================ 退出码 ================================

    0 = 符号正确（保持当前 gimbal_joint_sign）
    1 = 符号反了（按输出给的命令改 yaml）
    2 = 用法/环境错误
    3 = 没采到有效运动（链路没通、底盘没转够、或 TF 没在更新）—— 先排查, 别改符号
"""

import argparse
import math
import re
import sys

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import (ConnectivityException, ExtrapolationException,
                         LookupException)
except ImportError as exc:  # pragma: no cover - 环境问题
    sys.stderr.write(
        "[错误] 导入 ROS2 失败: %s\n"
        "       先 source /opt/ros/humble/setup.bash 和 install/setup.bash\n" % exc)
    sys.exit(2)

EXIT_OK = 0
EXIT_SIGN_WRONG = 1
EXIT_ENV = 2
EXIT_NO_MOTION = 3

MIN_EXCURSION_DEG = 5.0     # 低于这个幅度不给结论（防止把抖动当信号）
AGREE_TOL_DEG = 5.0         # θ_tf 与 θ_joint_states 的一致性容差
RATIO_TOL = 0.25            # |θ| 与 |deg| 的大小相对误差

HAND_TEXT = {
    'ccw': '底盘逆时针（俯视，车头往左转）',
    'cw': '底盘顺时针（俯视，车头往右转）',
}


def wrap_deg(angle):
    """折到 (-180, 180]。"""
    return (angle + 180.0) % 360.0 - 180.0


def unwrap(values):
    """把角度序列展开, 消除 ±180 跳变。"""
    out = []
    last = None
    offset = 0.0
    for value in values:
        if value is None:
            out.append(None)
            continue
        if last is None:
            out.append(value)
            last = value
            continue
        delta = wrap_deg(value - last)
        offset += delta - (value - last)
        out.append(value + offset)
        last = value
    return out


def peak_excursion(series):
    """返回 (峰值偏移, 峰值下标)。series 为展开后的角度序列, 以首值为基准。"""
    base = None
    best = 0.0
    best_i = None
    for i, value in enumerate(series):
        if value is None:
            continue
        if base is None:
            base = value
            continue
        dev = value - base
        if abs(dev) > abs(best):
            best = dev
            best_i = i
    return best, best_i


class SignProbe(Node):
    def __init__(self, args):
        super().__init__('check_gimbal_joint_sign')
        ns = args.ns.rstrip('/') if args.ns else ''

        self.buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.listener = TransformListener(self.buffer, self)

        self.deg_value = None
        self.deg_count = 0
        self.create_subscription(
            Float32, ns + '/' + args.deg_topic.lstrip('/'),
            self._on_deg, 10)

        self.joint_value = None
        self.joint_count = 0
        self.create_subscription(
            JointState, ns + '/joint_states', self._on_joint_states, 10)

        self.samples = []          # (t, deg, theta_tf, theta_joint)
        self.tf_errors = 0
        self.tf_ok = 0
        self.create_timer(1.0 / 30.0, self._tick)

        self.args = args

    def _on_deg(self, msg):
        self.deg_value = float(msg.data)
        self.deg_count += 1

    def _on_joint_states(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name == self.args.joint_name:
                self.joint_value = math.degrees(position)
                self.joint_count += 1
                break

    def _tick(self):
        theta_tf = None
        try:
            tf = self.buffer.lookup_transform(
                self.args.parent, self.args.child, Time())
            q = tf.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta_tf = math.degrees(math.atan2(siny, cosy))
            self.tf_ok += 1
        except (LookupException, ConnectivityException,
                ExtrapolationException):
            self.tf_errors += 1

        self.samples.append(
            (self.get_clock().now().nanoseconds / 1e9,
             self.deg_value, theta_tf, self.joint_value))


def analyze(probe, hand):
    samples = probe.samples
    deg = unwrap([s[1] for s in samples])
    theta_tf = unwrap([s[2] for s in samples])
    theta_js = unwrap([s[3] for s in samples])

    deg_peak, deg_i = peak_excursion(deg)
    tf_peak, tf_i = peak_excursion(theta_tf)
    js_peak, js_i = peak_excursion(theta_js)

    have_tf = tf_i is not None and abs(tf_peak) >= MIN_EXCURSION_DEG
    have_js = js_i is not None and abs(js_peak) >= MIN_EXCURSION_DEG
    have_deg = deg_i is not None and abs(deg_peak) >= MIN_EXCURSION_DEG

    print()
    print('-' * 62)
    print('采样结果（%d 个样本, TF 命中 %d / 失败 %d, deg 消息 %d, '
          'joint_states 命中 %d）'
          % (len(samples), probe.tf_ok, probe.tf_errors,
             probe.deg_count, probe.joint_count))
    if probe.deg_count:
        print('  /chassis_to_gimbal_yaw_deg（下位机） 峰值 %+7.2f°'
              % (deg_peak if have_deg else 0.0))
    if probe.tf_ok:
        print('  %s -> %s 的 yaw（URDF 关节角）  峰值 %+7.2f°  <- 判据看这条'
              % (probe.args.parent, probe.args.child,
                 tf_peak if have_tf else 0.0))
    if probe.joint_count:
        print('  /joint_states 的 %s 峰值 %+7.2f°'
              % (probe.args.joint_name, js_peak if have_js else 0.0))
    print('-' * 62)

    # ---------------- 环境/链路判定 ----------------
    if probe.tf_ok == 0:
        print('[FAIL] 查不到 %s -> %s 的 TF。' % (probe.args.parent,
                                                  probe.args.child))
        print('       base_link 的世界位姿是 sensor_scan_generation 反推出来的，'
              '没有这条 TF 就谈不上符号。')
        print('       先确认: ros2 run tf2_ros tf2_echo base_link gimbal_yaw')
        print('       常见原因: robot_state_publisher 没起 / URDF 里 '
              'gimbal_yaw_joint 被配成 fixed / relay 没发 gimbal_yaw_joint。')
        return EXIT_NO_MOTION

    if not have_tf:
        print('[FAIL] TF 有, 但 θ 的峰值只有 %.2f°（< %.0f°）, 不足以判断。'
              % (tf_peak if tf_i is not None else 0.0, MIN_EXCURSION_DEG))
        if probe.joint_count == 0:
            print('       /joint_states 里根本没有 %s —— relay 没起来, 或被 '
                  '--no-gimbal-joint-topic 钉成 0 了。'
                  % probe.args.joint_name)
        elif abs(js_peak) >= MIN_EXCURSION_DEG:
            print('       /joint_states 有 %+0.2f°, 但 TF 不跟着走: '
                  'robot_state_publisher 没吃这个关节。' % js_peak)
        else:
            print('       底盘没转够（云台锁住的前提下, 手转 30~45° 再转回来）, '
                  '或底盘断电导致 0x0010 停发。')
        return EXIT_NO_MOTION

    # ---------------- 方向判定 ----------------
    expect_theta_sign = -1 if hand == 'ccw' else +1
    actual_theta_sign = 1 if tf_peak > 0 else -1
    actual_deg_sign = None if not have_deg else (1 if deg_peak > 0 else -1)
    expect_deg_sign = 1 if hand == 'ccw' else -1

    print()
    print('你转的方向: %s' % HAND_TEXT[hand])
    print('按下位机口径本该: /chassis_to_gimbal_yaw_deg %s'
          % ('变大(+)' if expect_deg_sign > 0 else '变小(-)'))
    print('按 URDF 口径本该: θ = base_link -> gimbal_yaw %s'
          % ('变大(+)' if expect_theta_sign > 0 else '变小(-)'))
    print()

    sign_ok = actual_theta_sign == expect_theta_sign
    board_ok = (actual_deg_sign is None) or (actual_deg_sign == expect_deg_sign)

    if have_js:
        delta = abs(js_peak - tf_peak)
        same = delta <= AGREE_TOL_DEG
        print('  [%s] θ_tf 与 /joint_states 一致（差 %.2f°, 容差 %.0f°）'
              % ('OK  ' if same else 'WARN', delta, AGREE_TOL_DEG))
        if not same:
            print('         两条链不同源: 检查 relay 的 input_type 与 '
                  'joint_name, 或别处也在发 gimbal_yaw_joint。')

    if have_deg:
        # θ = sign × deg, 所以 θ 与 deg 应该反号
        cross_ok = actual_theta_sign == -actual_deg_sign
        print('  [%s] θ = sign × deg 的符号规律成立（deg %+0.1f° -> θ %+0.1f°）'
              % ('OK  ' if cross_ok else 'FAIL', deg_peak, tf_peak))
        ratio_err = abs(abs(tf_peak) - abs(deg_peak)) / max(abs(deg_peak), 1e-6)
        print('  [%s] 两者幅度相当（|θ|=%.1f°, |deg|=%.1f°, 相对差 %.0f%%, '
              '容差 %.0f%%）'
              % ('OK  ' if ratio_err <= RATIO_TOL else 'WARN',
                 abs(tf_peak), abs(deg_peak), ratio_err * 100, RATIO_TOL * 100))
        if not board_ok:
            print('  [WARN] 下位机的 deg 符号和文档口径**相反**（文档: 云台不动、'
                  '底盘逆时针 -> +）。')
            print('         这属于下位机侧口径问题, 但现场同样靠翻转 '
                  'gimbal_joint_sign 兜住。')
    else:
        print('  [WARN] 没收到 /%s, 只按 TF 判定。' % probe.args.deg_topic)

    print()
    if sign_ok:
        print('结论: 【符号正确】θ 跟着你的手反着走, base_link 在 RViz 里与底盘同向转。')
        print('      保持 %s 现在的值即可, 不用改。' % probe.args.param_file)
        return EXIT_OK

    print('结论: 【符号反了】θ 与底盘实际转向同号, base_link 会往反方向转。')
    print('      物理后果: 底盘速度方向误差翻倍, 比不接关节角更糟。')
    print()
    print('      改法:')
    print('        %s 第 %s 行, 翻转 gimbal_joint_sign'
          % (probe.args.param_file, probe.args.param_line or '?'))
    print('        cd <工作空间> && %s' % probe.args.flip_command)
    print('      改完不用重新编译（launch 直接读 yaml）, 重启导航后再跑一次本脚本复验。')
    print('      想先不落盘验证: ros2 run srm27_nav_protocol '
          'srm27_nav_protocol_node --ros-args -p gimbal_joint_sign:=%s'
          % flip_text(probe.args.current_sign))
    return EXIT_SIGN_WRONG


def read_param(path):
    """读 yaml 里 gimbal_joint_sign 的当前值与行号, 返回 (值, 行号)。"""
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            for lineno, line in enumerate(fh, 1):
                match = re.match(r'^\s*gimbal_joint_sign:\s*([-+0-9.]+)', line)
                if match:
                    return float(match.group(1)), lineno
    except OSError:
        pass
    return None, None


def resolve_param_file(path):
    """把 --param-file 解析成绝对路径（默认相对工作空间根）。"""
    import os
    if os.path.isabs(path):
        return path
    workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidate = os.path.join(workspace, path)
    if os.path.isfile(candidate):
        return candidate
    if os.path.isfile(path):
        return os.path.abspath(path)
    return candidate


def flip_text(sign):
    """翻转后的字面值, 用于打印命令。"""
    if sign is None:
        return '1.0（原值未知，当前若是 -1.0 就改 1.0，反之亦然）'
    return '-1.0' if sign > 0 else '1.0'


def main():
    parser = argparse.ArgumentParser(
        description='核对 gimbal_joint_sign（云台关节角符号）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='退出码: 0=符号正确 1=符号反了 2=环境问题 3=没采到有效运动')
    parser.add_argument('--hand', choices=['ccw', 'cw'], default='ccw',
                        help='你实际转动底盘的方向（俯视）: ccw=逆时针(默认), '
                             'cw=顺时针')
    parser.add_argument('--seconds', type=float, default=12.0,
                        help='采样时长（秒）, 默认 12')
    parser.add_argument('--ns', default='', help='命名空间, 如 /red_standard_robot1')
    parser.add_argument('--deg-topic', default='chassis_to_gimbal_yaw_deg',
                        help='下位机底盘相对云台角度话题（度）')
    parser.add_argument('--joint-name', default='gimbal_yaw_joint',
                        help='/joint_states 里的关节名')
    parser.add_argument('--parent', default='base_link', help='TF 父坐标系')
    parser.add_argument('--child', default='gimbal_yaw', help='TF 子坐标系')
    parser.add_argument('--param-file',
                        default='src/srm27_nav_protocol/config/'
                                'srm27_nav_protocol.yaml',
                        help='gimbal_joint_sign 所在的 yaml（相对工作空间根）')
    parser.add_argument('--no-pause', action='store_true',
                        help='不等待回车, 立刻开始采样（非交互环境用）')
    args = parser.parse_args()

    args.param_file = resolve_param_file(args.param_file)
    current_sign, args.param_line = read_param(args.param_file)
    args.current_sign = current_sign
    if args.param_line:
        args.flip_command = (
            "sed -i '%ds/^\\([[:space:]]*gimbal_joint_sign:\\).*/\\1 %s/'" %
            (args.param_line, flip_text(current_sign)))
    else:
        args.flip_command = ('sed -i "s/^\\([[:space:]]*gimbal_joint_sign:\\).*/'
                             '\\1 %s/" %s' % (flip_text(current_sign),
                                              args.param_file))

    rclpy.init()
    probe = SignProbe(args)
    try:
        print('=' * 62)
        print('核对 gimbal_joint_sign: 锁住云台, 手转底盘')
        print('=' * 62)
        print('  观察对象: θ = %s -> %s 的 yaw（就是 URDF 的 gimbal_yaw_joint）'
              % (args.parent, args.child))
        print('  正确表现: 底盘逆时针 -> θ 变负；底盘顺时针 -> θ 变正')
        print('  同时在 RViz 里: 红标 chassis_mark 跟着你的手转, '
              '蓝标 gimbal_mark 和雷达不动')
        print()

        deadline = probe.get_clock().now().nanoseconds / 1e9 + 5.0
        while probe.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
            if probe.tf_ok > 3 and probe.joint_count > 3:
                break
        if probe.tf_ok <= 3:
            print('[WARN] 5 s 内没抓到 %s -> %s 的 TF, 继续尝试（Ctrl-C 退出）'
                  % (args.parent, args.child))

        if not args.no_pause and sys.stdin.isatty():
            print()
            print('把车架起来 / 禁用驱动（**不要整车断电**, 断电 0x0010 停发, '
                  'TF 会冻结）。')
            print('云台锁住不动, 准备好后按回车开始 %0.0f s 采样...'
                  % args.seconds)
            try:
                input()
            except EOFError:
                pass
        print()
        print('>>> 现在开始: 慢速把底盘相对云台转 30~45°（%s）, 再转回来'
              % ('俯视逆时针' if args.hand == 'ccw' else '俯视顺时针'))

        end = probe.get_clock().now().nanoseconds / 1e9 + args.seconds
        while probe.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(probe, timeout_sec=0.05)
        print('<<< 采样结束')

        try:
            code = analyze(probe, args.hand)
        finally:
            pass
        return code
    except KeyboardInterrupt:
        print('\n[中断] 已取消, 未给结论。')
        return EXIT_NO_MOTION
    finally:
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
