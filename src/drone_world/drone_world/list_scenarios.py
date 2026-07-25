#!/usr/bin/env python3
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def main() -> None:
    config_dir = Path(get_package_share_directory('drone_world')) / 'config'
    print('Available drone_world scenarios:')
    for path in sorted(config_dir.glob('*.yaml')):
        print(f'- {path.stem}')


if __name__ == '__main__':
    main()
