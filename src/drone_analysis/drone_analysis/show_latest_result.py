#!/usr/bin/env python3

import json
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
            for path in results_root.glob('single_goal_*')
            if path.is_dir()
        ],
        reverse=True,
    )

    if not result_dirs:
        print('No single-goal result directory found.')
        return

    latest = result_dirs[0]
    summary_txt = latest / 'summary.txt'
    summary_json = latest / 'summary.json'

    print(f'Latest result: {latest}')
    print()

    if summary_txt.exists():
        print(summary_txt.read_text(encoding='utf-8'))
    elif summary_json.exists():
        print(
            json.dumps(
                json.loads(
                    summary_json.read_text(
                        encoding='utf-8'
                    )
                ),
                indent=2,
                ensure_ascii=False,
            )
        )
    else:
        print('No summary file found.')


if __name__ == '__main__':
    main()
