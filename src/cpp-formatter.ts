// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as vscode from "vscode";

/**
 * @link https://github.com/davetcoleman/roscpp_code_format
 */
const CLANG_FORMAT_STYLE = {
  BasedOnStyle: "Google",
  AccessModifierOffset: -2,
  ConstructorInitializerIndentWidth: 2,
  AlignEscapedNewlinesLeft: false,
  AlignTrailingComments: true,
  AllowAllParametersOfDeclarationOnNextLine: false,
  AllowShortIfStatementsOnASingleLine: false,
  AllowShortLoopsOnASingleLine: false,
  AllowShortFunctionsOnASingleLine: "None",
  AlwaysBreakTemplateDeclarations: true,
  AlwaysBreakBeforeMultilineStrings: true,
  BreakBeforeBinaryOperators: false,
  BreakBeforeTernaryOperators: false,
  BreakConstructorInitializersBeforeComma: true,
  BinPackParameters: true,
  ColumnLimit: 120,
  ConstructorInitializerAllOnOneLineOrOnePerLine: true,
  DerivePointerBinding: false,
  PointerBindsToType: true,
  ExperimentalAutoDetectBinPacking: false,
  IndentCaseLabels: true,
  MaxEmptyLinesToKeep: 1,
  NamespaceIndentation: "None",
  ObjCSpaceBeforeProtocolList: true,
  PenaltyBreakBeforeFirstCallParameter: 19,
  PenaltyBreakComment: 60,
  PenaltyBreakString: 1,
  PenaltyBreakFirstLessLess: 1000,
  PenaltyExcessCharacter: 1000,
  PenaltyReturnTypeOnItsOwnLine: 90,
  SpacesBeforeTrailingComments: 2,
  Cpp11BracedListStyle: false,
  Standard: "Auto",
  IndentWidth: 2,
  TabWidth: 2,
  UseTab: "Never",
  IndentFunctionDeclarationAfterType: false,
  SpacesInParentheses: false,
  SpacesInAngles: false,
  SpaceInEmptyParentheses: false,
  SpacesInCStyleCastParentheses: false,
  SpaceAfterControlStatementKeyword: true,
  SpaceBeforeAssignmentOperators: true,
  ContinuationIndentWidth: 4,
  SortIncludes: false,
  SpaceAfterCStyleCast: false,
  BreakBeforeBraces: "Custom",
  BraceWrapping: {
    AfterClass: true,
    AfterControlStatement: true,
    AfterEnum : true,
    AfterFunction : true,
    AfterNamespace : true,
    AfterStruct : true,
    AfterUnion : true,
    BeforeCatch : true,
    BeforeElse : true,
    IndentBraces : false
  }
};

/**
 * Formats C++ source using clang-format.
 */
export class CppFormatter implements vscode.DocumentFormattingEditProvider {
    public provideDocumentFormattingEdits(document: vscode.TextDocument,
        options: vscode.FormattingOptions,
        token: vscode.CancellationToken): Thenable<vscode.TextEdit[]> {
        const tabs = options.insertSpaces ? "Never" : "Always";
        const custom = { TabWidth: options.tabSize, UseTab: tabs };
        const style = JSON.stringify(Object.assign(CLANG_FORMAT_STYLE, custom));

        return new Promise((resolve, reject) => {
            const process = child_process.exec(`clang-format -style='${style}'`, (err, out) => {
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
