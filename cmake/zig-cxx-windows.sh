#!/bin/sh
exec zig c++ -target x86_64-windows-gnu "$@"
