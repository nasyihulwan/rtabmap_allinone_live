#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  nav_terminal.py — Terminal Interaktif Object-Goal Navigation   ║
║                                                                  ║
║  Tampilkan daftar objek dari semantic map, terima input class   ║
║  name dari user, publish ke /object_goal/target.               ║
║                                                                  ║
║  Usage:                                                          ║
║    python3 nav_terminal.py                                      ║
╚══════════════════════════════════════════════════════════════════╝
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SEMANTIC_MAP_PATH = os.path.expanduser('~/.ros/semantic_map.yaml')

CYAN   = '\033[0;36m'
GREEN  = '\033[0;32m'
YELLOW = '\033[1;33m'
RED    = '\033[0;31m'
BOLD   = '\033[1m'
NC     = '\033[0m'


def load_map(path):
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def parse_classes(semantic_map):
    """Ekstrak unique class names beserta jumlah instance-nya."""
    classes = {}
    for label in semantic_map:
        parts = label.rsplit('_', 1)
        if len(parts) == 2 and parts[1].isdigit():
            cls = parts[0]
            classes[cls] = classes.get(cls, 0) + 1
    return classes


def print_banner():
    print(f'\n{CYAN}{BOLD}╔══════════════════════════════════════════╗')
    print(f'║   Nav Terminal — Object-Goal Navigation  ║')
    print(f'╚══════════════════════════════════════════╝{NC}\n')


def print_objects(semantic_map):
    classes = parse_classes(semantic_map)
    if not classes:
        print(f'{YELLOW}⚠ Semantic map kosong atau belum ada objek.{NC}')
        return

    print(f'{BOLD}Objek tersedia ({len(classes)} class, {len(semantic_map)} instance):{NC}')
    for cls, count in sorted(classes.items()):
        instances = [l for l in semantic_map if l.rsplit('_', 1)[0] == cls]
        print(f'  {GREEN}•{NC} {cls:<25} ({count} instance: {", ".join(sorted(instances))})')
    print()


class NavTerminal(Node):
    def __init__(self):
        super().__init__('nav_terminal')
        self._pub = self.create_publisher(String, '/object_goal/target', 10)

    def send_target(self, class_name: str):
        msg = String()
        msg.data = class_name
        self._pub.publish(msg)
        self.get_logger().info(f'Published target: {class_name}')


def main():
    print_banner()
    print(f'{YELLOW}Tip: Ketik nama class (misal: shelf) → ke instance terdekat{NC}')
    print(f'{YELLOW}     Ketik instance label (misal: shelf_9) → ke instance spesifik{NC}')
    print(f'{YELLOW}     Ketik "list" untuk refresh daftar objek{NC}')
    print(f'{YELLOW}     Ketik "quit" atau Ctrl+C untuk keluar{NC}\n')

    # Load dan tampilkan map awal
    semantic_map = load_map(SEMANTIC_MAP_PATH)
    print_objects(semantic_map)

    print(f'{CYAN}Menginisialisasi ROS2...{NC}')
    rclpy.init()
    node = NavTerminal()
    print(f'{GREEN}✓ ROS2 ready. Siap menerima input.\n{NC}')

    try:
        while True:
            try:
                raw = input(f'{CYAN}{BOLD}>> Target class: {NC}').strip()
            except EOFError:
                break

            if not raw:
                continue

            if raw.lower() in ('quit', 'exit', 'q'):
                print(f'{YELLOW}Keluar.{NC}')
                break

            if raw.lower() in ('list', 'ls', 'l'):
                semantic_map = load_map(SEMANTIC_MAP_PATH)
                print_objects(semantic_map)
                continue

            # Validasi: cek class name ATAU instance label
            semantic_map = load_map(SEMANTIC_MAP_PATH)
            classes = parse_classes(semantic_map)

            # Cek exact instance label dulu
            if raw in semantic_map:
                parts = raw.rsplit('_', 1)
                cls = parts[0] if len(parts) == 2 else raw
                print(f'{GREEN}→ Navigasi ke instance "{raw}" (class: {cls})...{NC}')
                node.send_target(raw)
                print(f'{YELLOW}  Pantau terminal object_goal_nav untuk status navigasi.{NC}\n')
                continue

            # Cek class name
            if raw not in classes:
                print(f'{RED}✗ "{raw}" tidak ditemukan sebagai class maupun instance.{NC}')
                print(f'  Class tersedia: {", ".join(sorted(classes.keys()))}')
                print(f'  Instance tersedia: {", ".join(sorted(semantic_map.keys()))}\n')
                continue

            count = classes[raw]
            print(f'{GREEN}→ Navigasi ke "{raw}" ({count} instance, pilih terdekat)...{NC}')
            node.send_target(raw)
            print(f'{YELLOW}  Pantau terminal object_goal_nav untuk status navigasi.{NC}\n')

    except KeyboardInterrupt:
        print(f'\n{YELLOW}Keluar.{NC}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
