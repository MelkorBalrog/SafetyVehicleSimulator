REM -------------------------------------------------------------------------
REM This file is part of VDSS - Vehicle Dynamics Safety Simulator.
REM
REM VDSS is free software: you can redistribute it and/or modify
REM it under the terms of the GNU General Public License as published by
REM the Free Software Foundation, either version 3 of the License, or
REM (at your option) any later version.
REM
REM VDSS is distributed in the hope that it will be useful,
REM but WITHOUT ANY WARRANTY; without even the implied warranty of
REM MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
REM GNU General Public License for more details.
REM
REM You should have received a copy of the GNU General Public License
REM along with this program. If not, see <https://www.gnu.org/licenses/>.
REM -------------------------------------------------------------------------
@echo off
REM git_commit_and_merge.bat
REM Stages modified files, commits with a message, and merges into main.

setlocal enabledelayedexpansion

REM Determine current branch
for /f "usebackq tokens=*" %%b in (`git rev-parse --abbrev-ref HEAD`) do set "current_branch=%%b"
echo Current branch: %current_branch%

REM Stage all changes, including new files
echo Staging all changes...
git add -A
REM Check if there are staged changes; exit if none
git diff --cached --quiet
if errorlevel 1 goto DoCommit
echo No changes detected.
endlocal
goto :eof
:DoCommit

REM Commit changes
REM Commit changes: prompt for commit message
set /p commit_msg=Enter commit message: 
if "%commit_msg%"=="" (
    echo Commit message cannot be empty.
    endlocal
    exit /b 1
)
echo Committing with message: %commit_msg%
git commit -m "%commit_msg%"

REM Merge into main
if /I "%current_branch%" NEQ "main" (
    echo Checking out main branch...
    git checkout main
    echo Merging branch '%current_branch%' into main...
    git merge %current_branch%
)

git push origin main
echo Done.
endlocal