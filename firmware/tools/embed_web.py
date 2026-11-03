"""Pre-build step: gzips web/index.html into src/web_index.h so the page is
served straight from flash with Content-Encoding: gzip."""
import gzip
import os

Import("env")  # noqa: F821

root = env.subst("$PROJECT_DIR")  # noqa: F821
src = os.path.join(root, "web", "index.html")
dst = os.path.join(root, "src", "web_index.h")

with open(src, "rb") as f:
    data = gzip.compress(f.read(), compresslevel=9, mtime=0)

lines = [
    "/* Generated from web/index.html by tools/embed_web.py, do not edit. */",
    "#pragma once",
    "#include <stddef.h>",
    "#include <stdint.h>",
    f"static const size_t WEB_INDEX_GZ_LEN = {len(data)};",
    "static const uint8_t WEB_INDEX_GZ[] = {",
]
for i in range(0, len(data), 16):
    lines.append("    " + ", ".join(f"0x{b:02x}" for b in data[i:i + 16]) + ",")
lines += ["};", ""]
text = "\n".join(lines)

old = open(dst).read() if os.path.exists(dst) else None
if old != text:  # keep the mtime stable to avoid needless rebuilds
    with open(dst, "w") as f:
        f.write(text)
print(f"web UI: {len(data)} bytes gzipped")
