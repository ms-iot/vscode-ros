# Guidelines for Maintainers

## Index

- [Repository Policies](#repository-policies)
- [Project Workflow](#project-workflow)
- [Release Instructions](#release-instructions)

## Repository Policies

Please follow these principles for this repository:

- pull requests require 2+ approvals
- always work in forks
- do not merge your own changes
- follow [release instructions](#release-instructions)

### Working with this repository

Here are a few recommendations for maintainers of this project:

- While this project is created as a fork from the [original vscode-ros project][ajshort_vscode-ros], please **do not** merge `upstream/master` and push (unless planned).
- Please **do not** alter commit history unless it is necessary and everyone working on the project is notified:
    - **do not** use `git rebase`
    - **do not** use `git reset --hard` to revert to any commit earlier than current `HEAD`
    - try to avoid calling `git push --force`
- Please **try not to** directly push to `origin`, work with forks and merge changes through pull requests.
- Only use tags to track releases.

## Project workflow

This repository follows a simplified [Forking Workflow][forking_workflow] (an adaptation from the [Gitflow Workflow][gitflow_workflow]) to manage changes and branches.

1. the Gitflow Workflow works with 5 types of branches:
    - `master`: combined with tags to track all releases
    - `dev`: for current codebase development
    - `release`: branches off from `dev` for release-related cleanup and udpate
    - `feature`: branches off from `dev` for feature development
    - `hotfix`: branches off from `master` for product code fix
2. the Forking Workflow is based on the Gitflow Workflow yet most of the times without `feature` branches because:
    - `feature` branches are hosted in the developer's own fork, the centralized repository would not be hosting these branches
3. this repository also does not use the `master` branch (the `master` branch in this repository acts as the `dev` branch) because:
    - tags are adequate for tracking each release
    - `hotfix` branches can branch off from the latest tag instead of the `master` branch (this is manageable when the project does not have too many releases)

## Release Instructions

### Versioning

Versioning of this extension follows the [SemVer guidelines][semver_guidelines].

> Given a version number `MAJOR.MINOR.PATCH`, increment the:
>
> - `MAJOR` version when you make incompatible API changes,
> - `MINOR` version when you add functionality in a backwards-compatible manner, and
> - `PATCH` version when you make backwards-compatible bug fixes.
>
> Additional labels for pre-release and build metadata are available as extensions to the `MAJOR.MINOR.PATCH` format.

This project is not expected to have an insider's channel, so there are just some very simple guidelines to follow in practice:

1. when any change is introduced into the `master` branch after the existing release (at this point, the version number in the `master` branch should be `<current_version>`), update the version number in the `master` branch to `<new_version>-dev`
2. after branching off to a release branch (`release/*`), the version number in the release branch stays the same as `<new_version>-dev`
    - if any change comes into the `master` branch (instead of the release branch) at this point, the version number in the `master` branch should be updated to `<even_newer_version>-dev`
3. after the release branch has been reviewed and is ready to be released, update the version number to `<new_version>` and release
    - a newer version of the extension should be published as soon as the version number becomes `<new_version>`

Reasoning:

1. VS Code extension marketplace hosts only the latest version; when there is no insider's channel, there is only 1 public version (the latest version)
2. extensions can only be published with numeric SemVer versions, so no pre-release fields for final releases
3. in order for packages installed from `.vsix` published from the [vscode-ros.ci pipeline][vscode-ros.ci] to receive updates to the final release on the VS Code extension marketplace, the version number shipped in the `.vsix` package must be smaller. Therefore, the version numbers need to have the pre-release field (`-dev`)
4. since there is no insider's channel, which means pre-release builds installed from `.vsix` will not receive auto-update to another pre-release build, there is no further versioning for the pre-release fields (no `-alpha`, `-beta`, `-rc1`, `-rc2`, etc.)

### Publishing a release

#### Release checklist

Please review the following before publishing a new release:

Metadata:

- update `README.md`
- update `CHANGELOG.md`
- update version number in `package.json`

Release testing:

- Start, terminate, and monitor ROS core
    1. launch ROS core monitor with `ros.showMasterStatus`
    2. start a new ROS core process in background with `ros.startCore`
    3. check if ROS core monitor shows parameters and nodes correctly
    4. termintate the ROS core process with `ros.stopCore`
    5. check if ROS core monitor shows "offline"
- Create a terminal with ROS environment sourced
    1. start a new ROS terminal with `ros.createTerminal`
    2. check if ROS environment variables are properly configured
- Execute a ROS executable
    1. start a ROS core process (in another terminal or with `ros.startCore`)
    2. execute `ros.rosrun` and choose an executable
    3. check if the executable gets launched
- Execute a ROS launch file
    1. execute `ros.roslaunch` and choose a launch file
    2. check if the launch file gets launched

#### Authorizing a manual release (through the release pipeline)

The release process can be automated by triggering a release pipeline from the [vscode-ros.release pipeline][vscode-ros.release]; however, we will be simplifying the process to only release manually for now.

To authorize a release manually, schedule a release build with the proper release branch for the [vscode-ros.release pipeline][vscode-ros.release].

![schedule a release build][schedule_manual_release_build]

After the extension is released, the release branch should be:

1. tagged with version number
2. merged back into the `master` branch
3. deleted

#### Working with tags

Use [tags][git_tagging] to create snapshots of the codebase for each release. To create a new tag, follow these steps in a local `ms-iot/vscode-ros` git repository:

1. Sync with remote

    ```batch
    git pull
    ```

2. Create a new tag

    ```batch
    git tag --list
    git log --oneline -n <log_number>
    git tag <tag_name> <commit_id>
    git push origin <tag_name>
    ```

3. Remove a tag

    ```batch
    git tag --list
    git tag -d <tag_name>
    git push origin --delete <tag_name>
    ```

<!-- link to files -->
[schedule_manual_release_build]: /media/documentation/pipeline-manual-release.png

<!-- link to external sites -->
[ajshort_vscode-ros]: https://github.com/ajshort/vscode-ros
[forking_workflow]: https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow
[git_tagging]: https://git-scm.com/book/en/v2/Git-Basics-Tagging
[gitflow_workflow]: https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow
[semver_guidelines]: https://semver.org/#semantic-versioning-specification-semver
[vscode-ros.ci]: https://ros-win.visualstudio.com/ros-win/_build?definitionId=57
[vscode-ros.release]: https://ros-win.visualstudio.com/ros-win/_build?definitionId=68
