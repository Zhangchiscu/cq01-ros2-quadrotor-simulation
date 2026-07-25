#!/usr/bin/env python3

from pathlib import Path


def main() -> None:
    results_root = (
        Path.home()
        / 'drone_task'
        / 'your_drone_sim_ws'
        / 'results'
    )

    result_dirs = sorted(
        [
            path
            for path in results_root.glob(
                'multi_goal_*'
            )
            if path.is_dir()
        ],
        reverse=True,
    )

    if not result_dirs:
        print(
            'No multi-goal result directory found.'
        )
        return

    latest = result_dirs[0]
    summary = latest / 'summary.txt'

    print(f'Latest result: {latest}')
    print()

    if summary.exists():
        print(
            summary.read_text(
                encoding='utf-8'
            )
        )
    else:
        print('No summary.txt found.')


if __name__ == '__main__':
    main()
