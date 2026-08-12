#!/bin/sh
set -eu

c++ -std=c++17 -Wall -Wextra -Werror -pedantic -Isrc \
  src/protocol.cpp test/host_protocol/test_main.cpp -o /tmp/99l_ground_protocol_test
/tmp/99l_ground_protocol_test
