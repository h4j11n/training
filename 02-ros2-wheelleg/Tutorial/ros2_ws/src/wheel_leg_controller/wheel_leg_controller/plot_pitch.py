"""读取控制器保存的 pitch.csv，生成 PNG 图片。"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?', type=Path, default=Path('pitch.csv'))
    parser.add_argument('-o', '--output', type=Path, help='输出图片，默认与 CSV 同名，后缀为 .png')
    args = parser.parse_args()

    try:
        with args.csv.open(newline='', encoding='utf-8-sig') as file:
            rows = list(csv.DictReader(file))
        times = [float(row['sim_time_s']) for row in rows]
        pitches = [float(row['pitch_deg']) for row in rows]
    except (OSError, KeyError, ValueError) as exc:
        parser.error(f'无法读取 CSV（需要 sim_time_s 和 pitch_deg 两列）：{exc}')
    if not rows:
        parser.error('CSV 没有采样数据')
    if not all(math.isfinite(value) for value in times + pitches):
        parser.error('CSV 中存在无效的时间或 pitch 数值')

    fig, ax = plt.subplots(figsize=(10, 4.5), constrained_layout=True)
    ax.plot(times, pitches, color='#2563eb', linewidth=1.2, label='Pitch')
    ax.axhline(0, color='#64748b', linewidth=0.8)
    ax.axhline(10, color='#dc2626', linestyle='--', linewidth=1, label='+/-10 deg reference')
    ax.axhline(-10, color='#dc2626', linestyle='--', linewidth=1)
    ax.set_xlabel('Time since first sample (s)')
    ax.set_ylabel('Pitch (deg)')
    ax.set_title('Body pitch')
    ax.grid(True, alpha=0.25)
    ax.legend(loc='upper right')
    ax.margins(x=0)

    output = args.output or args.csv.with_suffix('.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(f'已生成图片：{output.resolve()}')


if __name__ == '__main__':
    main()
