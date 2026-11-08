#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""debug_tools 共用的小工具。

抽出来的都是"每个工具都要写一遍、又和具体业务无关"的样板：
matplotlib 配置目录、中文字体、CSV 落盘、`-h` 打印文件头注释、单位换算。

单独成文件而不是各工具复制一份，是为了让几个工具在环境差异上表现一致
（比如没有 `~/.config` 写权限、没有中文字体时的行为不该一个工具一个样）。
"""

import os
import sys
import tempfile
from pathlib import Path


def prepare_matplotlib_env():
    """MPLCONFIGDIR 默认在 ~/.config，容器/只读家目录下会刷警告，提前挪走。"""
    if os.environ.get("MPLCONFIGDIR"):
        return
    candidate = Path.home() / ".config" / "matplotlib"
    try:
        candidate.mkdir(parents=True, exist_ok=True)
        if os.access(candidate, os.W_OK):
            return
    except OSError:
        pass
    fallback = Path(tempfile.gettempdir()) / f"matplotlib-{os.getuid()}"
    try:
        fallback.mkdir(parents=True, exist_ok=True)
        os.environ["MPLCONFIGDIR"] = str(fallback)
    except OSError:
        pass


def setup_cjk_font():
    """图上要写中文，挑一个装了的 CJK 字体；找不到就返回空串（调用者提示）。"""
    from matplotlib import font_manager

    installed = {font.name for font in font_manager.fontManager.ttflist}
    for name in (
        "Noto Sans CJK JP", "Noto Sans CJK SC", "Noto Sans CJK TC",
        "Droid Sans Fallback", "WenQuanYi Zen Hei", "WenQuanYi Micro Hei",
    ):
        if name in installed:
            import matplotlib
            matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            # CJK 字体里没有 U+2212 的负号字形，负号会变方块
            matplotlib.rcParams["axes.unicode_minus"] = False
            return name
    return ""


def print_header_help(script_file):
    """把脚本文件头的注释块当成 --help 打出来（跳过 shebang 与编码声明）。"""
    with open(script_file, encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("#!") or "coding:" in line:
                continue
            if not line.startswith("#"):
                break
            print(line[2:].rstrip())


class CsvLog:
    """按行追加写 CSV：自动补表头，按行数定期 flush（被 kill -9 也不会丢太多）。

    写不进去（目录只读、磁盘满）只警告一次，不影响工具继续跑 —— 调试工具
    的首要任务是别在排查现场自己先崩。
    """

    def __init__(self, path, fields, flush_rows=25):
        self.path = Path(path)
        self.fields = tuple(fields)
        self.flush_rows = max(int(flush_rows), 1)
        self._handle = None
        self._rows = 0
        self._warned = False
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = self.path.open("w", newline="", encoding="utf-8")
            self._handle.write(",".join(self.fields) + "\n")
        except OSError as exc:
            self._handle = None
            self._warn_once(f"无法写 CSV {self.path}: {exc}")

    @property
    def enabled(self):
        return self._handle is not None

    def _warn_once(self, message):
        if not self._warned:
            self._warned = True
            print(f"[警告] {message}", file=sys.stderr)

    def write(self, row):
        if self._handle is None:
            return
        try:
            self._handle.write(",".join(row) + "\n")
            self._rows += 1
            if self._rows % self.flush_rows == 0:
                self._handle.flush()
        except OSError as exc:
            self._handle = None
            self._warn_once(f"写 CSV 失败，已停止记录: {exc}")

    def close(self):
        if self._handle is not None:
            try:
                self._handle.flush()
                self._handle.close()
            except OSError:
                pass
            self._handle = None


def fmt_bytes(count, unit="B"):
    """把字节数换成 1.2 GB 这种人类可读形式。"""
    value = float(count)
    for suffix in (unit, f"K{unit}", f"M{unit}", f"G{unit}", f"T{unit}"):
        if abs(value) < 1024.0 or suffix.startswith("T"):
            return f"{value:.1f}{suffix}" if suffix != unit else f"{int(value)}{suffix}"
        value /= 1024.0
    return f"{value:.1f}T{unit}"
