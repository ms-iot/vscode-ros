import * as cp from "child_process";
import * as vscode from "vscode";

/**
 * @link https://github.com/davetcoleman/roscpp_code_format
 */
const CLANG_FORMAT_STYLE = {
    AccessModifierOffset: -2,
    AlignEscapedNewlinesLeft: false,
    AlignTrailingComments: true,
    AllowAllParametersOfDeclarationOnNextLine: false,
    AllowShortFunctionsOnASingleLine: "None",
    AllowShortIfStatementsOnASingleLine: false,
    AllowShortLoopsOnASingleLine: false,
    AlwaysBreakBeforeMultilineStrings: false,
    AlwaysBreakTemplateDeclarations: true,
    BasedOnStyle: "Google",
    BinPackParameters: true,
    BreakBeforeBinaryOperators: false,
    BreakBeforeBraces: "Allman",
    BreakBeforeTernaryOperators: false,
    BreakConstructorInitializersBeforeComma: true,
    ColumnLimit: 120,
    ConstructorInitializerAllOnOneLineOrOnePerLine: true,
    ConstructorInitializerIndentWidth: 2,
    ContinuationIndentWidth: 4,
    Cpp11BracedListStyle: false,
    DerivePointerBinding: true,
    ExperimentalAutoDetectBinPacking: false,
    IndentCaseLabels: true,
    IndentFunctionDeclarationAfterType: false,
    IndentWidth: 2,
    MaxEmptyLinesToKeep: 1,
    NamespaceIndentation: "None",
    ObjCSpaceBeforeProtocolList: true,
    PenaltyBreakBeforeFirstCallParameter: 19,
    PenaltyBreakComment: 60,
    PenaltyBreakFirstLessLess: 1000,
    PenaltyBreakString: 1,
    PenaltyExcessCharacter: 1000,
    PenaltyReturnTypeOnItsOwnLine: 90,
    PointerBindsToType: false,
    SpaceAfterControlStatementKeyword: true,
    SpaceBeforeAssignmentOperators: true,
    SpaceInEmptyParentheses: false,
    SpacesBeforeTrailingComments: 2,
    SpacesInAngles: false,
    SpacesInCStyleCastParentheses: false,
    SpacesInParentheses: false,
    Standard: "Auto",
    TabWidth: 2,
    UseTab: "Never",
};

/**
 * Formats C++ source using clang-format.
 */
export default class CppFormatter implements vscode.DocumentFormattingEditProvider {
    public provideDocumentFormattingEdits(document: vscode.TextDocument,
        options: vscode.FormattingOptions,
        token: vscode.CancellationToken): Thenable<vscode.TextEdit[]> {
        const tabs = options.insertSpaces ? "Never" : "Always";
        const custom = { TabWidth: options.tabSize, UseTab: tabs };
        const style = JSON.stringify(Object.assign(CLANG_FORMAT_STYLE, custom));

        return new Promise((resolve, reject) => {
            const process = cp.exec(`clang-format -style='${style}'`, (err, out) => {
                if (!err) {
                    const lastLine = document.lineCount - 1;
                    const lastChar = document.lineAt(lastLine).text.length;
                    const range = new vscode.Range(0, 0, lastLine, lastChar);
                    const edit = vscode.TextEdit.replace(range, out);

                    resolve([edit]);
                } else {
                    reject(err);
                }
            });

            process.stdin.end(document.getText());
        });
    }
}
