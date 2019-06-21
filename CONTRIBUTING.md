# Contributing

This repository follows a simplified [Forking Workflow][forking_workflow] (an adaptation from the [Gitflow Workflow][gitflow_workflow]) to manage changes and branches. Detailed explanation could be found [here][maintainers_guidelines]. In practice, just follow these steps:

1. create your own fork of the repository
2. branch off from the latest `master` branch
3. finish changes in the feature branch
4. create pull request to the `master` branch and go through the review process

Done!

## Working with a fork

![a typical git fork][fork_repo]

For team-managed projects (like this one), even if you have the access to make changes directly through `git push`, it is still recommended to makes changes through pull requests so that all changes could be reviewed, verified, and clearly recorded as commits. To do that, follow the forking workflow used in most open-source projects.

### Creating a fork

After creating a fork of a repository, make sure to [configure a remote][git_configure_remote] first so the fork can sync up with the remote repository. For example, when working on a fork from this repository, do this first:

```batch
git add upstream https://github.com/ms-iot/vscode-ros
```

*Note: the remote name does not need to be `upstream`, we are just using `upstream` as an example*

### Syncing a fork

Forks do not automatically sync with the original repository, to keep forks up-to-date, here are a few ways to accomplish that:

1. `fetch + merge` as described in GitHub's [syncing a fork guide][git_sync_fork]

    ```batch
    git fetch -p upstream
    git checkout <branch>
    git merge upstream/<branch>
    ```

2. `pull upstream` (`git pull` is a wrapper for `fetch + merge`)

    ```batch
    git checkout <branch>
    git pull upstream <branch>
    ```

It is important to know that the above commands will only update the local git copy, the remote of the fork will not be updated. To keep both the local and the remote repositories up-to-date, make sure to do a `git push` after syncing.

<!-- ## Build Instructions -->

<!-- ## Contributing with a Pull Request -->

<!-- ## Coding Standards -->

<!-- ## Release Cycles-->

## Releasing a new version

Please check release instructions in [maintainers' guidelines][maintainers_guidelines].

## Microsoft Open Source Code of Conduct

This project welcomes contributions and suggestions. Most contributions require you to agree to a Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether you need to provide a CLA and decorate the PR appropriately (e.g., label, comment). Simply follow the instructions provided by the bot. You will only need to do this once across all repositories using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/). For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

<!-- link to files -->
[fork_repo]: /media/documentation/git-fork.png
[maintainers_guidelines]: MAINTAINERS_GUIDELINES.md

<!-- link to external sites -->
[forking_workflow]: https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow
[git_configure_remote]: https://help.github.com/en/articles/configuring-a-remote-for-a-fork
[git_sync_fork]: https://help.github.com/en/articles/syncing-a-fork
[gitflow_workflow]: https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow
