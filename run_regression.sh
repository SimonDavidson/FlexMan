#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Simon Davidson, University of Manchester
# Authors: Simon Davidson & Claude | Created 2026-06-07 | Last modified 2026-06-07
# =============================================================================
# run_regression.sh -- run every FlexMan testbench, report PASS/FAIL, exit code.
#
#   ./run_regression.sh                 # run all tests
#   ./run_regression.sh snnAcc          # run only tests whose path matches "snnAcc"
#   ./run_regression.sh --coverage      # also collect Xcelium code coverage
#   ./run_regression.sh --timeout=900   # per-test wall-clock limit (default 600s)
#
# Discovery: every *.bsh under the repo (excluding xcelium.d/) that declares an
# xrun "-top <module>".  Each is run in its own directory under `timeout`.
#
# Pass/fail is decided from the RUNTIME LOG (tests print one "PASS" but several
# conditional "FAIL" strings, so source counting is unreliable):
#   FAIL token present            -> FAIL            (takes precedence)
#   else PASS token present       -> PASS
#   else xcelium *E,/*F, present  -> FAIL(error)
#   else clean exit (rc 0)        -> PASS(elab)      (elaborate-only scripts)
#   else                          -> FAIL(rc=N)
# A `timeout` kill -> FAIL(timeout).
#
# Coverage (--coverage): an `xrun` shell-function wrapper appends -coverage args
# to every xrun invocation (no .bsh edits needed); databases land in cov_work/db
# and are merged with `imc` if it is on PATH (skipped gracefully otherwise).
# =============================================================================
set -u
ROOT="$(cd "$(dirname "$0")" && pwd)"
TIMEOUT=600
DO_COV=0
FILTER=""
for a in "$@"; do
    case "$a" in
        --coverage)   DO_COV=1 ;;
        --timeout=*)  TIMEOUT="${a#*=}" ;;
        --*)          echo "unknown option: $a" >&2; exit 64 ;;
        *)            FILTER="$a" ;;
    esac
done

LOGDIR="$ROOT/regress_logs"
rm -rf "$LOGDIR"; mkdir -p "$LOGDIR"
COVDB="$ROOT/cov_work/db"
[ "$DO_COV" -eq 1 ] && { rm -rf "$ROOT/cov_work"; mkdir -p "$COVDB"; }

# xrun wrapper: strips -gui (batch safety) and appends $COV_ARGS. Exported so it
# is visible inside the `bash -c` the runner uses to bound each test.
xrun() {
    local args=() a
    for a in "$@"; do [ "$a" = "-gui" ] || args+=("$a"); done
    command xrun "${args[@]}" ${COV_ARGS:-}
}
export -f xrun

mapfile -t TESTS < <(find "$ROOT" -name '*.bsh' -not -path '*/xcelium.d/*' \
                     -not -name 'selftest.bsh' -print0 2>/dev/null \
                     | xargs -0 grep -lE '\-top[[:space:]]+[A-Za-z0-9_]+' 2>/dev/null | sort)
# (selftest.bsh handled with the rest below; keep it in the suite)
mapfile -t SELF < <(find "$ROOT/verif" -name 'selftest.bsh' 2>/dev/null)
TESTS+=("${SELF[@]}")

npass=0; nfail=0
declare -a ROWS

printf '%s\n' "Running FlexMan regression (timeout ${TIMEOUT}s, coverage=${DO_COV})"
printf '%s\n' "-------------------------------------------------------------------"

for t in "${TESTS[@]}"; do
    [ -f "$t" ] || continue
    rel="${t#$ROOT/}"
    if [ -n "$FILTER" ] && [[ "$rel" != *"$FILTER"* ]]; then continue; fi
    dir="$(dirname "$t")"; base="$(basename "$t")"
    top="$(grep -oE '\-top[[:space:]]+[A-Za-z0-9_]+' "$t" | head -1 | awk '{print $2}')"
    tag="$(echo "$rel" | tr '/.' '__')"
    log="$LOGDIR/$tag.log"

    if [ "$DO_COV" -eq 1 ]; then
        export COV_ARGS="-coverage all -covworkdir $COVDB -covtest $tag -covoverwrite"
    else
        export COV_ARGS=""
    fi

    ( cd "$dir" && timeout "$TIMEOUT" bash -c 'source "$1"' _ "$base" ) >"$log" 2>&1
    rc=$?

    if   [ "$rc" -eq 124 ]; then status="FAIL(timeout)"
    elif grep -qE '(^|[^A-Za-z])FAIL([^A-Za-z]|$)' "$log"; then status="FAIL"
    elif grep -qE '(^|[^A-Za-z])PASS([^A-Za-z]|$)' "$log"; then status="PASS"
    elif grep -qE '\*[EF],' "$log"; then status="FAIL(error)"
    elif [ "$rc" -eq 0 ]; then status="PASS(elab)"
    else status="FAIL(rc=$rc)"
    fi

    case "$status" in
        PASS*) npass=$((npass+1)) ;;
        *)     nfail=$((nfail+1)) ;;
    esac
    ROWS+=("$(printf '%-44s %-14s %s' "$rel" "$status" "($top)")")
    printf '  %-44s %s\n' "$rel" "$status"
done

printf '%s\n' "-------------------------------------------------------------------"
printf 'SUMMARY: %d passed, %d failed, %d total\n' "$npass" "$nfail" "$((npass+nfail))"
if [ "$nfail" -gt 0 ]; then
    printf '\nFailures:\n'
    for r in "${ROWS[@]}"; do [[ "$r" == *FAIL* ]] && printf '  %s\n' "$r"; done
fi

# ---- coverage merge -------------------------------------------------------
if [ "$DO_COV" -eq 1 ]; then
    printf '\nCoverage databases under %s\n' "$COVDB"
    if command -v imc >/dev/null 2>&1; then
        cat > "$ROOT/cov_work/merge.tcl" <<TCL
merge ${COVDB}/* -out ${ROOT}/cov_work/merged -overwrite
load -run ${ROOT}/cov_work/merged
report -summary -inst -metric {block expression toggle fsm} \
       -out ${ROOT}/cov_work/report.txt -overwrite
exit
TCL
        imc -batch -exec "$ROOT/cov_work/merge.tcl" >"$ROOT/cov_work/imc.log" 2>&1 \
            && { printf 'Merged coverage report: cov_work/report.txt\n';
                 grep -iE 'overall|total|average' "$ROOT/cov_work/report.txt" | head -5; } \
            || printf 'imc merge failed (see cov_work/imc.log)\n'
    else
        printf 'imc not on PATH -- per-test coverage collected but not merged.\n'
    fi
fi

[ "$nfail" -eq 0 ]
