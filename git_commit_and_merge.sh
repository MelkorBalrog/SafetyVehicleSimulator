# -------------------------------------------------------------------------
# This file is part of VDSS - Vehicle Dynamics Safety Simulator.
#
# VDSS is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# VDSS is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------
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