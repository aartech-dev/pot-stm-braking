#!/usr/bin/env bash
# entrypoint.sh — route named commands or drop to shell
set -euo pipefail

CMD="${1:-bash}"
shift || true

case "${CMD}" in
    build)
        exec /workspace/scripts/build.sh "$@"
        ;;
    flash)
        exec /workspace/scripts/flash.sh "$@"
        ;;
    debug)
        exec /workspace/scripts/debug.sh "$@"
        ;;
    bash|sh)
        exec /bin/bash "$@"
        ;;
    *)
        # Pass anything else directly to bash
        exec /bin/bash -c "${CMD} $*"
        ;;
esac
