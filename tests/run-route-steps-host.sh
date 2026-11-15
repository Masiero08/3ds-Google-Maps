#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
${CC:-cc} -std=c11 -Wall -Wextra -Werror -pedantic -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Iinclude source/route_follow.c source/route_steps.c tests/host/route_steps_test.c -lm -o /tmp/route_steps_host
/tmp/route_steps_host
rm /tmp/route_steps_host
