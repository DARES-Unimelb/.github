"""Crude MATLAB syntax smoke test: bracket balance, comment/string awareness.

Not a parser. It catches the class of typo that would abort a run immediately:
unbalanced (), [], {}, an unterminated string, or a function file whose block
count does not close. Run from the workspace root:

    C:\\Python314\\python.exe results/control_syntax_check.py
"""

import sys
from pathlib import Path

ROOT = Path(r"C:\Ash\Projects\DARES_UACS\MATLAB")
FILES = sorted((ROOT / "control").glob("*.m"))

PAIRS = {")": "(", "]": "[", "}": "{"}


def check(path: Path) -> list[str]:
    problems: list[str] = []
    text = path.read_text(encoding="utf-8", errors="replace")
    depth = {"(": 0, "[": 0, "{": 0}
    stack: list[tuple[str, int]] = []
    in_str = False
    i = 0
    line = 1
    while i < len(text):
        ch = text[i]
        if ch == "\n":
            line += 1
            if in_str:
                problems.append(f"line {line - 1}: string not closed before newline")
                in_str = False
            i += 1
            continue
        if in_str:
            if ch == "'":
                if i + 1 < len(text) and text[i + 1] == "'":
                    i += 2
                    continue
                in_str = False
            i += 1
            continue
        if ch == "%":
            while i < len(text) and text[i] != "\n":
                i += 1
            continue
        if ch == "." and text[i : i + 3] == "...":
            while i < len(text) and text[i] != "\n":
                i += 1
            continue
        if ch == "'":
            # transpose if the previous non-space char is a value, else string open
            j = i - 1
            while j >= 0 and text[j] in " \t":
                j -= 1
            prev = text[j] if j >= 0 else ""
            if prev.isalnum() or prev in ")]}._":
                i += 1
                continue
            in_str = True
            i += 1
            continue
        if ch in "([{":
            depth[ch] += 1
            stack.append((ch, line))
        elif ch in ")]}":
            if not stack or stack[-1][0] != PAIRS[ch]:
                problems.append(f"line {line}: stray '{ch}'")
            else:
                stack.pop()
                depth[PAIRS[ch]] -= 1
        i += 1
    for opener, ln in stack:
        problems.append(f"line {ln}: '{opener}' never closed")
    if in_str:
        problems.append("file ends inside a string")

    # function / end balance, ignoring one-line anonymous functions and if/for blocks
    n_func = 0
    n_end = 0
    for ln, raw in enumerate(text.splitlines(), start=1):
        s = raw.split("%")[0].strip()
        if s.startswith("function"):
            n_func += 1
        if s == "end" or s.startswith("end;") or s.startswith("end "):
            n_end += 1
    if n_func and n_end < n_func:
        problems.append(f"functions {n_func} but only {n_end} plain 'end' lines")
    return problems


def main() -> int:
    bad = 0
    for path in FILES:
        problems = check(path)
        if problems:
            bad += 1
            print(f"FAIL {path.name}")
            for p in problems:
                print(f"   {p}")
        else:
            print(f"ok   {path.name}")
    print(f"{len(FILES)} files checked, {bad} with problems")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
