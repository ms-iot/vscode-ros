# Release Instructions

## Working with a fork
```
git pull upstream <branch>
git checkout -b <task_branch>
```

### Syncing tags with parent repo:
```
git fetch upstream
git push origin --tags
```

## Authorizing a manual release (through the release pipeline)


## Triggering an automatic release (with a new tag)
```
git pull
```

```
git tag --list
git log --oneline -n 10
git tag <tagname> fcd3f46
git push origin <tagname>
```

TODO: `git tag <tag>` only creates a lightweight tag. Signed tags are recommended for release purposes.

to remove a tag
```
git tag -d <tagname>
git push origin --delete <tagname>
```
