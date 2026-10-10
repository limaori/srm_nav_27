#!/usr/bin/env python3

# Copyright 2026 SRM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""速度限幅的单一来源：把 speed_limits.yaml 合并进 Nav2 / 底盘参数文件。

为什么需要它
------------
导航实际速度 = 链路上**所有**限幅层的最小值::

    控制器 FollowPath ─► velocity_smoother ─►[仿真] srm_cmd_mux ─► 执行端
                                          └►[实车] 串口 srm27_nav_protocol

改一个档位过去要同时改控制器、平滑器、（仿真还有）合成 mux，分散在 3~4 个文件里，
漏改任何一处，上游提速就会被下游静默吃掉（现场表现就是"改了参数没效果"）。
本模块把这几层收敛到每个 mode 一个文件：

    config/simulation/speed_limits.yaml
    config/real/speed_limits.yaml

launch 在启动时把它**深合并**进参数文件（覆盖层优先），参数文件里不再重复写这些键。

覆盖层语法
----------
顶层键就是 ROS 参数文件里的节点名，可加 `@变体` 后缀限定生效的控制器变体::

    controller_server@minco:      # 只在 MINCO 变体生效（合并时去掉 @minco）
      ros__parameters: {...}
    velocity_smoother@omni:      # 只在 Omni 变体生效
      ros__parameters: {...}
    srm_cmd_mux:                 # 不带 @ = 所有变体都生效（共用的节点）
      ros__parameters: {...}

变体由参数文件里 `controller_server.FollowPath.plugin` 判定（见 `detect_variant`）。
合并底盘控制（`srm_chassis_control.yaml`）这类没有控制器的参数文件时变体为空，
只有不带 `@` 的段生效。

"只此一处"是**强制**的
----------------------
若基础参数文件里也写了覆盖层里的同一个键，`merge_params()` 会把它算作冲突：

* 值不同 → 报错（`SpeedLimitError`）——那种"两处都在定义"正是要消灭的陷阱；
* 值相同 → 记为 duplicate，只告警（不阻断，但 `srm_speed_limits_check.py` 会报问题）。

想临时绕开整份覆盖层（例如手动传别的参数文件做实验），把 `speed_limits_file`
设为 `""`（或 `none`/`off`）即可，见 `DISABLED_VALUES`。
"""

import copy
import os
import tempfile

try:
    import yaml
except ImportError:  # pragma: no cover - ROS 环境一定有 PyYAML
    yaml = None

try:  # 允许在没有 launch 的场合（CLI 检查 / 单测）导入本模块
    import launch
    from launch.utilities import (
        normalize_to_list_of_substitutions,
        perform_substitutions,
    )

    # Humble 里 Substitution 挂在 launch 顶层（launch.substitutions 下没有这个名字），
    # nav2_common 的 RewrittenYaml 也是这么取的。
    Substitution = launch.Substitution
    HAVE_LAUNCH = True
except (ImportError, AttributeError):  # pragma: no cover
    HAVE_LAUNCH = False
    Substitution = object

#: `speed_limits_file` 取这些值时表示"不合并"，按原样使用参数文件。
DISABLED_VALUES = ("", "none", "off", "no", "false", "0")

#: `speed_limits_file` 取此值表示按约定自动查找（同目录的 speed_limits.yaml）。
AUTO = "auto"

#: 覆盖层的固定文件名。
OVERLAY_FILENAME = "speed_limits.yaml"

#: 只有名字以此开头的参数文件才参与自动查找，避免把限幅灌进上游参考文件。
SRM_PARAMS_PREFIX = "nav2_params_srm"

MINCO_PLUGIN = "srm27_minco_controller::MincoMpcController"
OMNI_PLUGIN = "srm27_omni_pid_controller::OmniPidPursuitController"

VARIANT_MINCO = "minco"
VARIANT_OMNI = "omni"
VARIANT_NONE = ""

#: Nav2 的节点段里这层是纯容器，比较路径时忽略。
_TRANSPARENT_KEYS = ("ros__parameters",)


class SpeedLimitError(RuntimeError):
    """覆盖层与参数文件冲突，或覆盖层本身不可用。"""


class MergeResult:
    """一次合并的结果。"""

    def __init__(
        self,
        merged,
        params_path=None,
        overlay_path=None,
        variant=VARIANT_NONE,
        sections=None,
        skipped_sections=None,
        conflicts=None,
        duplicates=None,
        merged_path=None,
    ):
        self.merged = merged
        self.params_path = params_path
        self.overlay_path = overlay_path
        self.variant = variant
        #: 实际生效的覆盖层顶层键（已去掉 @变体 后缀）。
        self.sections = sections or {}
        #: 因变体不匹配被跳过的覆盖层键。
        self.skipped_sections = skipped_sections or []
        #: 基础文件里也写了、且**值不同**的键（错误）。
        self.conflicts = conflicts or []
        #: 基础文件里也写了、值相同的键（应删除的冗余）。
        self.duplicates = duplicates or []
        #: 合并后写出的临时文件（未写出时为 None）。
        self.merged_path = merged_path

    @property
    def applied(self):
        return bool(self.sections)


def load_yaml(path):
    """读 YAML；文件缺失或为空都返回 {}。"""
    if yaml is None:  # pragma: no cover
        raise SpeedLimitError("需要 PyYAML（python3-yaml）才能读取参数文件")
    try:
        with open(path, "r") as stream:
            data = yaml.safe_load(stream)
    except OSError as exc:
        raise SpeedLimitError("无法读取 %s: %s" % (path, exc)) from exc
    except yaml.YAMLError as exc:
        raise SpeedLimitError("解析 %s 失败: %s" % (path, exc)) from exc
    return data if isinstance(data, dict) else {}


def _strip_containers(node):
    """穿透 `ros__parameters` 这类纯容器键。"""
    section = node
    while isinstance(section, dict):
        inner = None
        for candidate in _TRANSPARENT_KEYS:
            if isinstance(section.get(candidate), dict):
                inner = section[candidate]
                break
        if inner is None:
            break
        section = inner
    return section


def _node_section(cfg, node):
    """取某个节点的参数段；找不到返回 None。"""
    if not isinstance(cfg, dict):
        return None
    if isinstance(cfg.get(node), dict):
        return _strip_containers(cfg[node])
    for value in cfg.values():
        found = _node_section(value, node)
        if found is not None:
            return found
    return None


_MISSING = object()


def _lookup_path(node, parts):
    """按路径逐段下钻，途中穿透容器键与重复的节点名。

    Nav2 的代价地图段写作 `local_costmap: local_costmap: ros__parameters:`（节点名重复
    一次），控制器写作 `controller_server: ros__parameters:`；两种都要能走通。顺序是
    "先按字面匹配下一段，再穿透 ros__parameters，最后吃掉重复的节点名"。
    """
    if not parts:
        return node
    if not isinstance(node, dict):
        return _MISSING

    if parts[0] in node:
        found = _lookup_path(node[parts[0]], parts[1:])
        if found is not _MISSING:
            return found

    for container in _TRANSPARENT_KEYS + (parts[0],):
        if isinstance(node.get(container), dict):
            found = _lookup_path(node[container], parts)
            if found is not _MISSING:
                return found
    return _MISSING


def _lookup(cfg, path):
    """按 `.` 分隔的路径取值（穿透容器键 / 重复节点名）；不存在返回 None。"""
    found = _lookup_path(cfg, path.split("."))
    return None if found is _MISSING else found


#: 公开别名：检查脚本 / 预检脚本也用同一条取值路径（穿透 ros__parameters / 重复节点名）。
lookup = _lookup


def detect_variant(cfg):
    """按 `controller_server.FollowPath.plugin` 判定控制器变体。"""
    plugin = _lookup(cfg, "controller_server.FollowPath.plugin")
    if not isinstance(plugin, str):
        return VARIANT_NONE
    if MINCO_PLUGIN in plugin:
        return VARIANT_MINCO
    if OMNI_PLUGIN in plugin:
        return VARIANT_OMNI
    return VARIANT_NONE


def split_section_key(key):
    """把 `节点名@变体` 拆成 (节点名, 变体)；不带 @ 时变体为 None。"""
    if "@" in key:
        node, _, variant = key.partition("@")
        return node, variant.strip().lower()
    return key, None


def effective_sections(overlay, variant):
    """按变体挑出要生效的覆盖层段，返回 ({节点名: 段}, [被跳过的键])。"""
    sections = {}
    skipped = []
    for key, value in overlay.items():
        node, wanted = split_section_key(key)
        if wanted is None or wanted == variant:
            sections[node] = value
        else:
            skipped.append(key)
    return sections, skipped


def _flatten(node, prefix=""):
    """把参数段压成 {点分路径: 标量或列表}。"""
    leaves = {}
    for key, value in node.items():
        if key in _TRANSPARENT_KEYS and isinstance(value, dict):
            leaves.update(_flatten(value, prefix))
            continue
        path = ("%s.%s" % (prefix, key)) if prefix else str(key)
        if isinstance(value, dict):
            leaves.update(_flatten(value, path))
        else:
            leaves[path] = value
    return leaves


def _same_value(lhs, rhs):
    if isinstance(lhs, bool) or isinstance(rhs, bool):
        return lhs == rhs
    if isinstance(lhs, (int, float)) and isinstance(rhs, (int, float)):
        return abs(float(lhs) - float(rhs)) < 1e-9
    return lhs == rhs


def _deep_merge_into(base, overlay, prefix=""):
    """把 overlay 深合并进 base（就地修改并返回）；列表整体替换，不逐项合并。"""
    for key, value in overlay.items():
        path = ("%s.%s" % (prefix, key)) if prefix else str(key)
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_into(base[key], value, path)
        else:
            base[key] = copy.deepcopy(value)
    return base


def merge_params(
    base_cfg,
    overlay_cfg,
    variant=None,
    allow_overrides=False,
    base_label="参数文件",
    overlay_label=OVERLAY_FILENAME,
):
    """把覆盖层合并进参数文件（不改动入参）。

    :param base_cfg: 参数文件内容（dict）。
    :param overlay_cfg: 覆盖层内容（dict）。
    :param variant: 强制指定变体；None 表示按 base_cfg 自动判定。
    :param allow_overrides: 冲突只告警不报错（默认报错）。
    :param base_label / overlay_label: 报错信息里显示的文件名。
    :returns: MergeResult。
    """
    if variant is None:
        variant = detect_variant(base_cfg)
    sections, skipped = effective_sections(overlay_cfg, variant)

    # 覆盖层的段必须与参数文件同层：Nav2 / 底盘参数文件都是
    # `<节点名>: ros__parameters: {...}`，覆盖层少了这层容器就会合并到一个
    # 节点根本不会读的键上（静默失效），因此这里直接拦下来。
    for node, section in sections.items():
        base_node = base_cfg.get(node) if isinstance(base_cfg, dict) else None
        if not isinstance(base_node, dict) or not isinstance(section, dict):
            continue
        if "ros__parameters" in base_node and "ros__parameters" not in section:
            raise SpeedLimitError(
                "覆盖层的 %s 段缺少 ros__parameters 容器（%s 里是 "
                "`%s: ros__parameters: {...}`）；请照抄参数文件的层级。"
                % (node, base_label, node)
            )

    conflicts = []
    duplicates = []
    for node, section in sections.items():
        base_section = _node_section(base_cfg, node)
        if base_section is None:
            continue
        base_leaves = _flatten(base_section)
        for path, value in _flatten(section).items():
            if path not in base_leaves:
                continue
            entry = "%s.%s" % (node, path)
            if _same_value(base_leaves[path], value):
                duplicates.append(entry)
            else:
                conflicts.append(
                    "%s.%s: %s 里是 %s，覆盖层是 %s"
                    % (node, path, base_label, base_leaves[path], value)
                )

    merged = copy.deepcopy(base_cfg)
    for node, section in sections.items():
        merged.setdefault(node, {})
        _deep_merge_into(merged[node], section)

    result = MergeResult(
        merged=merged,
        variant=variant,
        sections=sections,
        skipped_sections=skipped,
        conflicts=conflicts,
        duplicates=duplicates,
    )
    if conflicts and not allow_overrides:
        raise SpeedLimitError(
            "速度限幅在两处都有定义，且值不一致：\n  - %s\n"
            "速度限幅只允许写在 %s 里：请把 %s 里的这些键删掉。"
            "（覆盖层优先的静默合并，正是『改了没效果』的来源。）"
            % ("\n  - ".join(conflicts), overlay_label, base_label)
        )
    return result


def resolve_overlay_path(params_path, explicit=AUTO):
    """决定用哪份覆盖层。

    * `explicit` 为 None/"auto"：同目录下存在 `speed_limits.yaml`，且参数文件是
      `nav2_params_srm*.yaml` 时自动使用它（上游参考文件不参与，避免把 SRM 限幅
      灌进别的配置），否则返回 None。
    * `explicit` 为 DISABLED_VALUES 之一：明确不合并。
    * 其它：按给定路径（必须存在）。
    """
    if explicit is None:
        explicit = AUTO
    spec = os.path.expanduser(str(explicit)).strip()
    if spec.lower() in DISABLED_VALUES:
        return None
    if spec.lower() != AUTO:
        if not os.path.isfile(spec):
            raise SpeedLimitError("speed_limits_file 指向的文件不存在: %s" % spec)
        return os.path.abspath(spec)
    if not os.path.basename(params_path).startswith(SRM_PARAMS_PREFIX):
        return None
    candidate = os.path.join(
        os.path.dirname(os.path.abspath(params_path)), OVERLAY_FILENAME
    )
    return candidate if os.path.isfile(candidate) else None


def merge_params_files(params_path, overlay=AUTO, allow_overrides=False):
    """读参数文件 + 覆盖层并合并（不写盘）。"""
    if not os.path.isfile(params_path):
        raise SpeedLimitError("参数文件不存在: %s" % params_path)
    base_cfg = load_yaml(params_path)
    overlay_path = resolve_overlay_path(params_path, overlay)
    if overlay_path is None:
        return MergeResult(
            merged=base_cfg,
            params_path=params_path,
            variant=detect_variant(base_cfg),
        )
    result = merge_params(
        base_cfg,
        load_yaml(overlay_path),
        variant=None,
        allow_overrides=allow_overrides,
        base_label=os.path.basename(params_path),
        overlay_label=os.path.basename(overlay_path),
    )
    result.params_path = params_path
    result.overlay_path = overlay_path
    return result


def write_merged(result, prefix="srm_speed_limits_"):
    """把合并结果写成临时 YAML（与 nav2 RewrittenYaml 的做法一致），返回路径。"""
    handle = tempfile.NamedTemporaryFile(
        mode="w", delete=False, prefix=prefix, suffix=".yaml"
    )
    with handle:
        yaml.safe_dump(
            result.merged,
            handle,
            default_flow_style=False,
            sort_keys=False,
            allow_unicode=True,
        )
    result.merged_path = handle.name
    return handle.name


def _first(value):
    if isinstance(value, (list, tuple)) and value:
        return value[0]
    return value


def chain_summary(cfg):
    """抽出"这条链实际生效的限幅值"，用于打印与自检。"""
    follow = "controller_server.FollowPath"
    controller = _lookup(cfg, follow + ".v_linear_max")
    if controller is None:
        controller = _lookup(cfg, follow + ".limits.max_linear_speed")
    summary = {
        "controller": _first(controller),
        "controller_accel": _first(_lookup(cfg, follow + ".limits.max_linear_accel")),
        "smoother": _first(_lookup(cfg, "velocity_smoother.max_velocity")),
        "smoother_accel": _first(_lookup(cfg, "velocity_smoother.max_accel")),
        "mux_linear": _first(_lookup(cfg, "srm_cmd_mux.v_max")),
        "behavior_max_rotational_vel": _first(
            _lookup(cfg, "behavior_server.max_rotational_vel")
        ),
    }
    layers = [
        value
        for value in (summary["controller"], summary["smoother"], summary["mux_linear"])
        if isinstance(value, (int, float)) and value > 0.0
    ]
    summary["binding"] = min(layers) if layers else None
    return summary


def format_chain(cfg, indent="    "):
    """把生效链打印成多行文本（给 launch 日志与检查脚本用）。"""

    def _fmt(value, unit):
        return "%.2f %s" % (value, unit) if isinstance(value, (int, float)) else "—"

    summary = chain_summary(cfg)
    binding = summary["binding"]
    lines = [
        "%s控制器 FollowPath: %s（加速度 %s）"
        % (
            indent,
            _fmt(summary["controller"], "m/s"),
            _fmt(summary["controller_accel"], "m/s²"),
        ),
        "%s速度平滑器:       %s（加速度 %s）"
        % (
            indent,
            _fmt(summary["smoother"], "m/s"),
            _fmt(summary["smoother_accel"], "m/s²"),
        ),
    ]
    if summary["mux_linear"] is not None:
        lines.append(
            "%s合成 srm_cmd_mux:  %s"
            % (indent, _fmt(summary["mux_linear"], "m/s 合速度"))
        )
    lines.append(
        "%s⇒ 这条链的上限 = %s（取各层最小）"
        % (indent, "%.2f m/s" % binding if binding is not None else "—")
    )
    return "\n".join(lines)


if HAVE_LAUNCH:

    class SpeedLimitParams(Substitution):
        """launch 用的替换：把参数文件与速度限幅覆盖层合并成一个临时文件。

        用法::

            configured = ParameterFile(
                RewrittenYaml(
                    source_file=SpeedLimitParams(params_file, speed_limits_file),
                    ...
                )
            )

        `overlay_file` 取 `auto` 时按 `resolve_overlay_path` 的约定自动查找；
        取 `""` 时不合并（原样返回参数文件路径）。
        """

        def __init__(
            self, base_file, overlay_file=AUTO, allow_overrides=False, quiet=False
        ):
            super().__init__()
            self.__base = normalize_to_list_of_substitutions(base_file)
            self.__overlay = normalize_to_list_of_substitutions(overlay_file)
            self.__allow_overrides = allow_overrides
            self.__quiet = quiet
            self.__cache = {}
            #: 首次成功合并产出的文件；之后的任何求值都复用它。
            self.__merged_path = None

        @property
        def name(self):
            return self.__base

        def describe(self):
            return "速度限幅覆盖层合并"

        def perform(self, context):
            base_path = perform_substitutions(context, self.__base)
            overlay_spec = perform_substitutions(context, self.__overlay)
            cache_key = (os.path.abspath(base_path), str(overlay_spec))
            if cache_key in self.__cache:
                return self.__cache[cache_key]
            # launch 会把同一个替换在子上下文里再求值一次（include 的 launch_arguments
            # 是替换对象），那时 params_file 已经被上游重写成 /tmp 临时文件，
            # "按文件名认 mode" 的自动查找必然落空。既然已经产出过合并文件，就直接复用。
            if self.__merged_path is not None:
                self.__cache[cache_key] = self.__merged_path
                return self.__merged_path

            result = merge_params_files(
                base_path, overlay_spec, allow_overrides=self.__allow_overrides
            )
            if not result.applied:
                self._report_without_overlay(base_path, str(overlay_spec))
                self.__cache[cache_key] = base_path
                return base_path
            merged_path = write_merged(result)
            self.__merged_path = merged_path
            if not self.__quiet:
                header = "[srm_speed_limits] 合并 %s + %s（变体 %s）" % (
                    os.path.basename(base_path),
                    os.path.basename(result.overlay_path),
                    result.variant or "共用段",
                )
                print("%s\n%s" % (header, format_chain(result.merged)))
                if result.duplicates:
                    print(
                        "    重复定义（参数文件里应删除）: %s"
                        % ", ".join(result.duplicates)
                    )
                print("    合并文件: %s" % merged_path)
            self.__cache[cache_key] = merged_path
            return merged_path

        def _report_without_overlay(self, base_path, overlay_spec):
            """没有合并时只报"用户看得懂"的那两种情况，其余保持安静。"""
            if self.__quiet:
                return
            if overlay_spec.strip().lower() in DISABLED_VALUES:
                print(
                    "[srm_speed_limits] %s：已按要求关闭速度限幅合并"
                    "（speed_limits_file=%r）。\n"
                    "    注意：这不是『回到参数文件里的旧值』，而是退回**插件/节点默认值**"
                    "（Omni 控制器默认 v_linear_max=3.0 m/s，比档位高）；"
                    "要临时换档位，请另存一份 speed_limits.yaml 并显式指定路径。"
                    % (os.path.basename(base_path), overlay_spec)
                )
                return
            # auto 且参数文件看起来就是 SRM 那份，却没找到同目录的覆盖层 —— 这值得提醒。
            if os.path.basename(base_path).startswith(SRM_PARAMS_PREFIX):
                print(
                    "[srm_speed_limits] 警告: %s 同目录下没有 %s，"
                    "没有合并任何速度限幅（限速将以参数文件里的值为准）"
                    % (os.path.basename(base_path), OVERLAY_FILENAME)
                )
