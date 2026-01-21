# Complete GitHub Workflow — ACE MiniProject

This document records the complete Git and GitHub workflow used in the ACE MiniProject.  
It is written as a single continuous procedure, following the real order in which Git commands are used during development.  
The purpose of this document is to allow any collaborator to reproduce the workflow without external explanations.

The repository is organised such that the Git repository root is the folder `ACE/`. Inside this repository, the PlatformIO project is located in the subfolder `ACE/MiniProject/`, and all documentation is stored in `ACE/docs/`. All Git commands shown in this document are executed from the repository root (`ACE/`). Embedded development is done by opening `ACE/MiniProject/` in VS Code.

Every work session starts by synchronising the local repository with GitHub. This step ensures that the local machine has the most recent version of the project and avoids conflicts before any new work begins. The first command used is `git status`, which shows the current branch and whether there are uncommitted changes. This allows verification that the working directory is clean before proceeding.

```bash
git status
git checkout main
git pull
git checkout -b branchname
git add -A
git add .nameofthefile
git commit -m "messages"
git push -u origin branchname~
git checkout main
git pull
git merge feature/sensors
git push
git branch -d feature/sensors




