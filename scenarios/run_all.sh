#!/bin/bash
# Run every scenario headless with 5 seeds each and write reports/scenario_results.md.
#   scenarios/run_all.sh                 # all scenarios, seeds 1-5, 3 Webots at a time
#   JOBS=1 scenarios/run_all.sh          # one at a time
#   SEEDS="1 2" scenarios/run_all.sh     # fewer seeds
#   scenarios/run_all.sh mine side_pit   # only these scenarios
# Exit code 0 only if every run passed.
cd "$(dirname "$0")/.." || exit 1
ONLY=()
[ $# -gt 0 ] && ONLY=(--only "$@")
exec .venv/bin/python scenarios/run_scenario.py --all --seeds ${SEEDS:-1 2 3 4 5} --jobs "${JOBS:-3}" "${ONLY[@]}"
