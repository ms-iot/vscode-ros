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
2. after the `master` branch has been reviewed and is ready to be released, update the version number to `<new_version>` and release
    - a newer version of the extension should be published as soon as the version number becomes `<new_version>`
3. then, go to step 1.

Reasoning:

1. VS Code extension marketplace hosts only the latest version; when there is no insider's channel, there is only 1 public version (the latest version)
2. extensions can only be published with numeric SemVer versions, so no pre-release fields for final releases
3. in order for packages installed from `.vsix` published from the [vscode-ros.ci pipeline][vscode-ros.ci] to receive updates to the final release on the VS Code extension marketplace, the version number shipped in the `.vsix` package must be smaller. Therefore, the version numbers need to have the pre-release field (`-dev`)
4. since there is no insider's channel, which means pre-release builds installed from `.vsix` will not receive auto-update to another pre-release build, there is no further versioning for the pre-release fields (no `-alpha`, `-beta`, `-rc1`, `-rc2`, etc.)

### Publishing a release

#### Testing before Release

Before preparing a release, you should check the health of this extension, for example,

1. Check the latest CI status:
   ![.github/workflows/workflow.yml](https://github.com/ms-iot/vscode-ros/workflows/.github/workflows/workflow.yml/badge.svg?event=push)

2. Run through the basic scenarios manually on Linux and Windows environments.
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
    - Execute URDF preview on a URDF file.
    - Configure ROS launch debug configuration, set breakpoints and start debugging.

#### Preparing a Release

For each release, we will need to bump the version number and update the documents accordingly.

The following will be touched:

- update `README.md`
- update `CHANGELOG.md`
- update version number in `package.json` and `package-lock.json`

> You can find an example commit [here](https://github.com/ms-iot/vscode-ros/commit/3091180d319d2ca87736cd50c1293dd26151a0b8).

#### Authorizing a release (through GitHub releases)

The release process is integrated and automated with [GitHub releases](https://docs.github.com/en/github/administering-a-repository/about-releases). Here are 3-easy steps to kick off one.

1. [Draft a new release](https://github.com/ms-iot/vscode-ros/releases/new).

2. Fill the version number and release note. (Keep the target branch as `master`.)
   ![](/media/documentation/draft-release.png)

3. Click `Publish release`.

Once it is kicked off, GitHub release will automatically tag your latest commit from the `master` and a GitHub action will automatically be triggered to build and publish the extension to VS Code extension marketplace. For example, here is [one release run](https://github.com/ms-iot/vscode-ros/actions/runs/580197093) in the past.

#### Post-release Activities

To get `master` ready for the next release, it is encouraged to update the version number immediately after a release is published. You can find an example commit [here](https://github.com/ms-iot/vscode-ros/commit/3fd13ba1def4f0eee3a0fc9e0e58db7558e119a3).


<!-- link to external sites -->
[ajshort_vscode-ros]: https://github.com/ajshort/vscode-ros
[forking_workflow]: https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow
[git_tagging]: https://git-scm.com/book/en/v2/Git-Basics-Tagging
[gitflow_workflow]: https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow
[semver_guidelines]: https://semver.org/#semantic-versioning-specification-semver
[vscode-ros.ci]: https://github.com/ms-iot/vscode-ros/actions
