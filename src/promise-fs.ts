import * as fs from "fs";

function call(fn: Function, ...args: any[]): Promise<any> {
    return new Promise((c, e) => fn(...args, (err, res) => err ? e(err) : c(res)));
}

export function exists(path: string): Promise<boolean> {
    return new Promise(c => fs.exists(path, c));
}

export function readFile(filename: string, encoding: string): Promise<string>;
export function readFile(filename: string): Promise<Buffer>;

export function readFile(filename: string, encoding?: string) {
    return call(fs.readFile, ...arguments);
}

export function readdir(path: string): Promise<string[]> {
    return call(fs.readdir, path);
}

export function mkdir(path: string): Promise<void> {
    return call(fs.mkdir, path);
}

export function writeFile(filename: string, data: any): Promise<void> {
    return call(fs.writeFile, filename, data);
}
