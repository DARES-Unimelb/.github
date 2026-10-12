"""Static consistency checks for the Simulink layer, runnable without MATLAB.

Why this exists: the three failure modes below only surface inside MATLAB, and
every trial run costs a slice of the single shared session. The checks are cheap
enough to run before every build.

  1. every function file balances function/end
  2. parentheses and brackets balance on every logical line, continuation lines
     joined first
  3. every sim_* name used as a block parameter in a builder is pushed into the
     base workspace by sim_push_design_vars or sim_push_scenario
  4. every add_line endpoint names a block that the same builder creates, and no
     builder adds the same block name twice (that is the
     "A new block named ... cannot be added" failure)

Usage:
   python sim_static_check.py            checks <MATLAB root>/sim, run, verify, ...

Exit status is 0 when clean, 1 when a problem was found. Known false positive:
a regular expression inside a string literal that contains unbalanced brackets,
such as the token pattern in the sizing probe job.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIRS = ["sim", "run", "verify", "params", "plant", "control"]

openers = re.compile(r"^\s*(function|if|for|while|switch|try|parfor)\b")
closers = re.compile(r"^\s*end\b")


def strip_comment(line):
    """Drop a trailing comment, keeping quoted strings intact."""
    out = []
    in_str = False
    i = 0
    while i < len(line):
        c = line[i]
        if c == "'":
            if in_str and i + 1 < len(line) and line[i + 1] == "'":
                out.append("''")
                i += 2
                continue
            in_str = not in_str
            out.append(c)
            i += 1
            continue
        if c == "%" and not in_str:
            break
        out.append(c)
        i += 1
    return "".join(out)


def logical_lines(lines):
    """Join continuation lines so brackets can be counted per statement."""
    buf = []
    start = 0
    for n, raw in enumerate(lines, 1):
        line = strip_comment(raw)
        if not buf:
            start = n
        stripped = line.rstrip()
        if stripped.endswith("..."):
            buf.append(stripped[:-3])
            continue
        buf.append(stripped)
        yield start, " ".join(buf)
        buf = []
    if buf:
        yield start, " ".join(buf)


def check_balance(path):
    problems = []
    depth = 0
    stack = []
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    for n, line in logical_lines(lines):
        if not line.strip():
            continue
        if openers.match(line):
            depth += 1
            stack.append(n)
        elif closers.match(line):
            if depth == 0:
                problems.append(f"{path.name}:{n}: 'end' with no opener")
            else:
                depth -= 1
                stack.pop()
        for o, c in (("(", ")"), ("[", "]")):
            if line.count(o) != line.count(c):
                problems.append(
                    f"{path.name}:{n}: unbalanced {o}{c} "
                    f"({line.count(o)} vs {line.count(c)}): {line[:90]}"
                )
    if depth != 0:
        problems.append(f"{path.name}: {depth} unclosed block(s), opened at {stack}")
    return problems


def collect_pushed():
    pushed = set()
    for name in ("sim_push_design_vars.m", "sim_push_scenario.m"):
        text = (ROOT / "sim" / name).read_text(encoding="utf-8")
        pushed |= set(re.findall(r"assignin\('base',\s*'([A-Za-z_]\w*)'", text))
        pushed |= set(re.findall(r"local_push\(written,\s*'([A-Za-z_]\w*)'", text))
        # sim_pid_<loop>_<suffix> names are built with sprintf
        pushed |= set(re.findall(r"sprintf\('sim_pid_%s_(\w+)'", text))
    return pushed


def used_block_params():
    sfun_names = {"sim_plant_sfun", "sim_meas_sfun", "sim_pilot_sfun", "sim_alloc_sfun"}
    used = {}
    for name in ("sim_build_plant.m", "sim_build_manual_pid.m", "sim_build_lqr.m",
                 "sim_add_pid.m", "sim_add_plant_subsystem.m", "sim_new_model.m"):
        text = (ROOT / "sim" / name).read_text(encoding="utf-8")
        for m in re.finditer(r"'([A-Za-z_]\w*)'", text):
            tok = m.group(1)
            if tok in sfun_names or not tok.startswith("sim_"):
                continue
            if tok.startswith("sim_build") or tok.startswith("sim_add"):
                continue
            used.setdefault(tok, set()).add(name)
        for m in re.finditer(r"sim_pid_%s_(\w+)", text):
            used.setdefault("sim_pid_<loop>_" + m.group(1), set()).add(name)
    return used


def main():
    problems = []
    files = []
    for d in DIRS:
        p = ROOT / d
        if p.is_dir():
            files += sorted(p.glob("*.m"))
    for f in files:
        problems += check_balance(f)
    print(f"checked {len(files)} .m files for block and bracket balance")

    pushed = collect_pushed()
    used = used_block_params()
    missing = []
    for tok, where in sorted(used.items()):
        if tok.startswith("sim_pid_<loop>_"):
            suffix = tok.replace("sim_pid_<loop>_", "")
            for loop in ("p", "q", "r", "phi", "theta", "V", "h"):
                if f"sim_pid_{loop}_{suffix}" not in pushed:
                    missing.append(f"{tok} (loop {loop}) used in {sorted(where)}")
        elif tok not in pushed:
            missing.append(f"{tok} used in {sorted(where)}")
    print(f"pushed sim_* variables: {len(pushed)}")
    if missing:
        print("MISSING PUSHES:")
        for m in missing:
            print("  " + m)
    else:
        print("all sim_* block parameters have a pusher")

    for name in ("sim_build_plant.m", "sim_build_manual_pid.m", "sim_build_lqr.m",
                 "sim_add_plant_subsystem.m"):
        text = (ROOT / "sim" / name).read_text(encoding="utf-8")
        added = re.findall(r"sim_add_block\((?:\w+),\s*'([^']+)'", text)
        added += re.findall(r"sim_add_pid\((?:\w+),\s*'([^']+)'", text)
        added += re.findall(r"sim_add_plant_subsystem\((?:\w+),\s*'([^']+)'", text)
        dups = sorted({b for b in added if added.count(b) > 1})
        if dups:
            problems.append(f"{name}: duplicate block names {dups}")
        names = set(added)
        endpoints = {m.group(1) for m in re.finditer(r"'([^'/]+)/\d+'", text)}
        unknown = sorted(e for e in endpoints
                         if e not in names and e not in {"e", "Kp", "Ki", "Kd", "sum", "u"})
        if unknown:
            problems.append(f"{name}: connect endpoints with no block: {unknown}")

    if problems:
        print(f"\n{len(problems)} PROBLEM(S):")
        for p in problems:
            print("  " + p)
        return 1
    print("\nno structural problems found")
    return 0


if __name__ == "__main__":
    sys.exit(main())
