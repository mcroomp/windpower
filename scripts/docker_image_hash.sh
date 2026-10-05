#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

{
    for path in \
        scripts/docker_image_hash.sh \
        simulation/Dockerfile \
        simulation/requirements.txt \
        simulation/requirements-docker.txt \
        linkhub/Cargo.toml \
        linkhub/Cargo.lock
    do
        printf '%s\0' "$path"
        sha256sum "$REPO_DIR/$path"
    done

    while IFS= read -r -d '' path; do
        relative="${path#"$REPO_DIR/"}"
        printf '%s\0' "$relative"
        sha256sum "$path"
    done < <(find "$REPO_DIR/linkhub/src" -type f -print0 | sort -z)
} | sha256sum | awk '{print $1}'
