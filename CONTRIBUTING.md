# Contributing

Please follow general work flow while contributing to this repository.

## Working with a fork

![a typical git fork][fork_repo]

When working with team-managed projects (like this one), it is generally considered better practice not to make changes directly (through `git push`). Instead, changes are better made through pull requests so that all changes could be reviewed, verified, and clearly recorded as commits.

### Creating a fork

After creating a fork of a repository, make sure to [configure a remote][git_configure_remote] first. For example, when working on a fork from this repository, do this first:

```
git remote -v
git add upstream https://github.com/ms-iot/vscode-ros
```

### Syncing a fork

Forks do not automatically sync with the original repository, to keep forks up-to-date, here are a few ways to accomplish that:

1. `fetch + merge` as described in GitHub's [syncing a fork guide][git_sync_fork]
    ```
    git fetch upstream
    git checkout <branch>
    git merge upstream/<branch>
    git checkout -b <new_branch>
    ```

2. `pull upstream` (`git pull` is a wrapper for `fetch + merge`)
    ```
    git checkout <branch>
    git pull upstream <branch>
    git checkout -b <new_branch>
    ```

It is important to know that the above commands will only update the local git copy, the remote of the fork will not be updated. To keep both the local and the remote repositories up-to-date, make sure to do a `git push` after syncing.

### How to make life easier

If you're like me and don't like to delete and create new forks every time, then syncing a fork with remote will become one of the most regular tasks for you too. To save time and effort on this, it is recommended to:
- never touch branches from the remote repository (`master`, etc.) so syncing could be as easy as
    ```
    git pull upstream <branch> && git push
    ```
- always create a new branch for local work (so branches from the remote repositories will not be touched)

### Syncing tags with parent repo:

While tags are very similar to branches (tags are stored as `refs/tags/<tag>` and branches are stored as `refs/heads/<branch>`), syncing are very different when it comes to forks.

The normal `git pull upstream <branch>` will not be sufficient. To keep the fork's tags up-to-date with the remote, use the following commands:
```
git fetch upstream
git push origin --tags
```

<!-- ## Build Instructions

## Generating a `dev` Build

## Contributing with a Pull Request

## Coding Standards

## Release Cycles-->

## Releasing a new version

Please check [release instructions][maintainers_guidelines].

## Microsoft Open Source Code of Conduct

This project welcomes contributions and suggestions. Most contributions require you to agree to a Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether you need to provide a CLA and decorate the PR appropriately (e.g., label, comment). Simply follow the instructions provided by the bot. You will only need to do this once across all repositories using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

<!-- link to files -->
[fork_repo]: /media/documentation/git-fork.png
[maintainers_guidelines]: MAINTAINERS_GUIDELINES.md

<!-- link to external sites -->
[git_configure_remote]: https://help.github.com/en/articles/configuring-a-remote-for-a-fork
[git_sync_fork]: https://help.github.com/en/articles/syncing-a-fork
