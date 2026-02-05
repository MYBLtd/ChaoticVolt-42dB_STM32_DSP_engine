#!/usr/bin/env bash
set -euo pipefail

ELF="${1:-build/42dB_DSP_Engine.elf}"

gdb-multiarch -q "$ELF" \
  -ex "set architecture armv7e-m" \
  -ex "target remote localhost:13333" \
  -ex "monitor reset halt" \
  -ex "load" \
  -ex "monitor reset run" \
  -ex "detach" \
  -ex "quit"

