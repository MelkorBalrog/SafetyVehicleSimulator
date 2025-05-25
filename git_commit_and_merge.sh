#!/usr/bin/env bash
set -euo pipefail

# git_commit_and_merge.sh
# Finds modified files, stages them, commits with a message, and merges into main.

# Determine current branch
current_branch=$(git rev-parse --abbrev-ref HEAD)
echo "Current branch: $current_branch"

# Stage all changes, including new files
echo "Staging all changes..."
git add -A
# Check if there are staged changes; exit if none
if git diff --cached --quiet; then
  echo "No changes detected."
  exit 0
fi

# Commit changes: prompt for commit message
git config --global user.email "miguel.marina.g@gmail.com"
read -rp "Enter commit message: " commit_msg
if [ -z "$commit_msg" ]; then
  echo "Commit message cannot be empty."
  exit 1
fi
echo "Committing with message: $commit_msg"
git commit -m "$commit_msg"

# Merge into main if not already on main branch
if [ "$current_branch" != "main" ]; then
  echo "Checking out main branch..."
  git checkout main
  echo "Merging branch '$current_branch' into main..."
  git merge "$current_branch"
fi

git push origin main
echo "Done."