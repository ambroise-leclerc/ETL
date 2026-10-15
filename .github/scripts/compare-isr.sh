#!/usr/bin/env bash
# Compares the interrupt code that two AVR ELF files install on the given vectors.
# Usage: compare-isr.sh <a.elf> <b.elf> <vector number>...
# For each vector, follows the jump in the vector table and prints the handler's instructions up to
# the first reti, without addresses or symbol names, then diffs the two listings.
set -euo pipefail

isr_code() {
    local elf=$1 vector=$2
    avr-objdump -d --no-show-raw-insn "$elf" | awk -v vector="$vector" '
        /<__vectors>:/ { in_table = 1; next }
        in_table && /jmp/ {
            if (entry++ == vector) { match($0, /0x[0-9a-f]+ </); target = substr($0, RSTART + 2, RLENGTH - 4); in_table = 0 }
            next
        }
        in_table { in_table = 0 }
        target != "" && $1 == target ":" { dumping = 1 }
        dumping && /^[ ]+[0-9a-f]+:/ {
            sub(/^[ ]+[0-9a-f]+:[ \t]+/, ""); sub(/[ \t]*;.*$/, ""); print
            if ($1 == "reti") exit
        }'
}

a=$1 b=$2
shift 2
status=0
for vector in "$@"; do
    code_a=$(isr_code "$a" "$vector")
    code_b=$(isr_code "$b" "$vector")
    if [ -z "$code_a" ] || [ -z "$code_b" ]; then
        echo "vector $vector: no handler found"; status=1
    elif [ "$code_a" != "$code_b" ]; then
        echo "vector $vector: different code"; diff <(echo "$code_a") <(echo "$code_b") || true; status=1
    else
        echo "vector $vector: identical ($(echo "$code_a" | wc -l) instructions)"
    fi
done
exit $status
