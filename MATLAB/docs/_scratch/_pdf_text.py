import sys, re
import pymupdf

path = sys.argv[1]
out = sys.argv[2]
max_pages = int(sys.argv[3]) if len(sys.argv) > 3 else 40

doc = pymupdf.open(path)
n = min(len(doc), max_pages)
chunks = []
for i in range(n):
    t = doc[i].get_text()
    chunks.append(f"===== PAGE {i+1} =====\n{t}")
with open(out, "w", encoding="utf-8") as f:
    f.write(f"FILE: {path}\nPAGES: {len(doc)} (extracted {n})\n\n")
    f.write("\n".join(chunks))
print(f"OK pages={len(doc)} extracted={n} -> {out}")
