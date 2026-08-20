#!/usr/bin/env bash
# Kill processes matching a pattern WITHOUT killing the shell doing the killing.
#
# `pkill -f <pattern>` matches full command lines, and an interactive shell's command line
# almost always contains the pattern it was asked to search for -- so pkill kills its own
# caller. That cost several restarts before it was noticed. This skips anything that looks
# like the agent's own shell, and skips self and parents explicitly.
set -u
pattern="$1"
skipped=0 killed=0
me=$$
ancestors=" $me "
p=$me
while p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' '); [ -n "$p" ] && [ "$p" != 0 ]; do
    ancestors="$ancestors$p "
done
for pid in $(pgrep -f "$pattern" 2>/dev/null); do
    case "$ancestors" in *" $pid "*) skipped=$((skipped+1)); continue ;; esac
    cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null) || continue
    case "$cmd" in
        *shell-snapshots*|*claude*|*safe_kill.sh*) skipped=$((skipped+1)); continue ;;
    esac
    kill -9 "$pid" 2>/dev/null && killed=$((killed+1))
done
echo "safe_kill '$pattern': killed $killed, skipped $skipped"
