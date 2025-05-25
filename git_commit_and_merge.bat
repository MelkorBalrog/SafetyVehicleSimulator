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