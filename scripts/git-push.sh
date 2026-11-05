#!/usr/bin/env bash
# Push without IDE git askpass. Prompts once for GitHub username + PAT.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=git-env.sh
source "$REPO/scripts/git-env.sh"
cd "$REPO"
git -c credential.helper=store push "$@"
