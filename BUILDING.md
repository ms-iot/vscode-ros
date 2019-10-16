You can set up a development environment for debugging the extension during extension development.

## Building and Debugging the extension

Ensure you have [node](https://nodejs.org/en/) installed.
Clone the repo, run `npm install` and open a development instance of Code.

```bash
git clone https://github.com/Microsoft/vscode-ros
cd vscode-ros
npm install
code .
```
Make sure the `window.openFoldersInNewWindow` setting is not `"on"`.

You can now go to the Debug viewlet (`Ctrl+Shift+D`) and select `Launch Extension` then hit run (`F5`).

This will open a new VS Code window which will have the title `[Extension Development Host]`. In this window, open any folder with Go code.

In the original VS Code window, you can now add breakpoints which will be hit when you use any of the the plugin's features in the second window.

If you make edits in the extension `.ts` files, just reload (`cmd-r`) the `[Extension Development Host]` instance of Code to load in the new extension code.  The debugging instance will automatically reattach.

## Running the tests
To run the tests locally, open the Debug viewlet (`Ctrl+Shift+D`), select `Launch Tests`, then hit run (`F5`)

## Sideloading the extension
After making changes to the extension, you might want to test it end to end instead of running it in debug mode. To do this, you can sideload the extension. This can be done by preparing the extension and loading it directly.

1. `npm install -g vsce` to make sure you have vsce installed globally
2. `git clone https://github.com/Microsoft/vscode-ros` to clone the repo if you havent already done so
3. `cd vscode-ros`
4. `npm install` to install dependencies if you havent already done so
5. `vsce package` to build the package. This will generate a file with extension `vsix`
6. Run the command `Extensions: Install from VSIX...`, choose the vsix file generated in the previous step
