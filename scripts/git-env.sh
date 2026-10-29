# Source from repo root:  source scripts/git-env.sh
# Bypasses VS Code/Cursor git integration (broken in plain Brev/SSH shells).
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GIT_CONFIG_SYSTEM="${GIT_CONFIG_SYSTEM:-$REPO_ROOT/scripts/git-empty-system.config}"
# Empty global config so /root or IDE ~/.gitconfig credential helpers are not loaded.
export GIT_CONFIG_GLOBAL="${GIT_CONFIG_GLOBAL:-$REPO_ROOT/scripts/git-empty-global.config}"

# IDE terminals inject these; they route auth to a dead unix socket, not your shell.
unset GIT_ASKPASS SSH_ASKPASS
unset VSCODE_GIT_ASKPASS_NODE VSCODE_GIT_ASKPASS_MAIN VSCODE_GIT_ASKPASS_EXTRA_ARGS
unset VSCODE_GIT_ASKPASS_PIPE VSCODE_GIT_ASKPASS_TYPE VSCODE_GIT_IPC_HANDLE
unset VSCODE_GIT_EDITOR_NODE VSCODE_GIT_EDITOR_MAIN VSCODE_GIT_EDITOR_EXTRA_ARGS

export GIT_TERMINAL_PROMPT=1
