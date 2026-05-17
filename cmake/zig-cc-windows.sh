#!/bin/sh
exec zig cc -target x86_64-windows-gnu "$@"
