#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -pthread -Itests/host -Iinclude \
    tests/process_host_test.cpp src/logger.cpp src/state_machine.cpp -o "$out/process_test"
"$out/process_test"
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -Iinclude -fsyntax-only tests/imu_fifo_test.cpp

"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -pthread -Itests/host -Iinclude \
    tests/imu_stop_host_test.cpp src/imu.cpp -o "$out/imu_stop_test"
"$out/imu_stop_test"
