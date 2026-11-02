import sys, io
from docx import Document
from docx.table import Table
from docx.text.paragraph import Paragraph
from docx.oxml.ns import qn

path = sys.argv[1]
out = sys.argv[2]
doc = Document(path)

def iter_block_items(parent):
    from docx.document import Document as _Doc
    if isinstance(parent, _Doc):
        parent_elm = parent.element.body
    else:
        parent_elm = parent._tc
    for child in parent_elm.iterchildren():
        if child.tag == qn('w:p'):
            yield Paragraph(child, parent)
        elif child.tag == qn('w:tbl'):
            yield Table(child, parent)

lines = []
body = doc.element.body
idx = 0
pi = 0
ti = 0
for blk in iter_block_items(doc):
    if isinstance(blk, Paragraph):
        style = blk.style.name if blk.style is not None else ''
        pPr = blk._p.pPr
        num = ''
        if pPr is not None and pPr.numPr is not None:
            num = ' NUM'
        runs_info = []
        for r in blk.runs:
            f = r.font
            runs_info.append(f"[{r.text!r} b={f.bold} sz={f.size.pt if f.size else None} name={f.name}]")
        lines.append(f"P{pi:03d} <{style}>{num} ALIGN={blk.alignment} :: {blk.text}")
        if runs_info:
            lines.append("     runs: " + " ".join(runs_info))
        pi += 1
    else:
        lines.append(f"TABLE{ti}: rows={len(blk.rows)} cols={len(blk.columns)} style={blk.style.name if blk.style else None}")
        for ri, row in enumerate(blk.rows):
            cells = [c.text.replace('\n', ' / ') for c in row.cells]
            lines.append("  | " + " | ".join(cells))
        ti += 1

lines.append("")
lines.append("=== sections ===")
for s in doc.sections:
    lines.append(f"page {s.page_width}x{s.page_height} margins L{s.left_margin} R{s.right_margin} T{s.top_margin} B{s.bottom_margin}")
lines.append("=== inline shapes ===")
for sh in doc.inline_shapes:
    lines.append(f"shape type={sh.type} w={sh.width} h={sh.height}")
lines.append("=== core props ===")
cp = doc.core_properties
for a in ('title', 'author', 'subject', 'comments', 'last_modified_by'):
    lines.append(f"{a}: {getattr(cp, a)}")

with io.open(out, 'w', encoding='utf-8') as f:
    f.write("\n".join(lines))
print("done", len(lines))
