#!/bin/sh
exec zig c++ -target x86_64-linux-gnu.2.28 "$@"
