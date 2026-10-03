#!/usr/bin/env python3
"""
Tom & Jerry SNES BETA <-> final static WRAM auto-mapper.

Compares two 512 KiB LoROM builds, aligns equivalent ROM code, follows
reachable 65C816 code from the interrupt/reset vectors, and extracts RAM
address operands that changed between builds.

No external Python packages are required.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import struct
from collections import Counter, defaultdict, deque
from pathlib import Path

# 65C816 opcodes with fixed 16-bit absolute data operands.
ABS16_OPS = {
    0x0C:"TSB",0x0D:"ORA",0x0E:"ASL",0x1C:"TRB",0x1D:"ORA",0x1E:"ASL",
    0x2C:"BIT",0x2D:"AND",0x2E:"ROL",0x3C:"BIT",0x3D:"AND",0x3E:"ROL",
    0x4D:"EOR",0x4E:"LSR",0x5D:"EOR",0x5E:"LSR",
    0x6D:"ADC",0x6E:"ROR",0x7D:"ADC",0x7E:"ROR",
    0x8C:"STY",0x8D:"STA",0x8E:"STX",0x99:"STA",0x9C:"STZ",0x9D:"STA",0x9E:"STZ",
    0xAC:"LDY",0xAD:"LDA",0xAE:"LDX",0xB9:"LDA",0xBC:"LDY",0xBD:"LDA",0xBE:"LDX",
    0xCC:"CPY",0xCD:"CMP",0xCE:"DEC",0xD9:"CMP",0xDD:"CMP",0xDE:"DEC",
    0xEC:"CPX",0xED:"SBC",0xEE:"INC",0xF9:"SBC",0xFD:"SBC",0xFE:"INC",
}

LONG24_OPS = {
    0x0F:"ORA",0x1F:"ORA",0x2F:"AND",0x3F:"AND",
    0x4F:"EOR",0x5F:"EOR",0x6F:"ADC",0x7F:"ADC",
    0x8F:"STA",0x9F:"STA",0xAF:"LDA",0xBF:"LDA",
    0xCF:"CMP",0xDF:"CMP",0xEF:"SBC",0xFF:"SBC",
}

# 65C816 instruction lengths. 'M' and 'X' mean immediate width depends
# on the processor M/X status bits.
_ROWS = {
0x0:[2,2,2,2,2,2,2,2,1,'M',1,1,3,3,3,4],
0x1:[2,2,2,2,2,2,2,2,1,3,1,1,3,3,3,4],
0x2:[3,2,4,2,2,2,2,2,1,'M',1,1,3,3,3,4],
0x3:[2,2,2,2,2,2,2,2,1,3,1,1,3,3,3,4],
0x4:[1,2,2,2,3,2,2,2,1,'M',1,1,3,3,3,4],
0x5:[2,2,2,2,3,2,2,2,1,3,1,1,4,3,3,4],
0x6:[1,2,3,2,2,2,2,2,1,'M',1,1,3,3,3,4],
0x7:[2,2,2,2,2,2,2,2,1,3,1,1,3,3,3,4],
0x8:[2,2,3,2,2,2,2,2,1,'M',1,1,3,3,3,4],
0x9:[2,2,2,2,2,2,2,2,1,3,1,1,3,3,3,4],
0xA:['X',2,'X',2,2,2,2,2,1,'M',1,1,3,3,3,4],
0xB:[2,2,2,2,2,2,2,2,1,3,1,1,3,3,3,4],
0xC:['X',2,2,2,2,2,2,2,1,'M',1,1,3,3,3,4],
0xD:[2,2,2,2,2,2,2,2,1,3,1,1,3,3,3,4],
0xE:['X',2,2,2,2,2,2,2,1,'M',1,1,3,3,3,4],
0xF:[2,2,2,2,3,2,2,2,1,3,1,1,3,3,3,4],
}
INSN_LEN = [1] * 256
for hi, vals in _ROWS.items():
    for lo, val in enumerate(vals):
        INSN_LEN[(hi << 4) | lo] = val

VECTOR_OFFSETS = [0x7FE4,0x7FE6,0x7FE8,0x7FEA,0x7FEE,0x7FF4,0x7FF8,0x7FFA,0x7FFC,0x7FFE]


def md5(data: bytes) -> str:
    return hashlib.md5(data).hexdigest()


def strip_copier_header(data: bytes):
    if len(data) % 0x8000 == 512:
        return data[512:], 512
    return data, 0


def title(data: bytes) -> str:
    if len(data) >= 0x7FD5:
        return data[0x7FC0:0x7FD5].decode("ascii", "replace").rstrip()
    return ""


def lorom_cpu_to_off(cpu: int, size: int):
    bank = (cpu >> 16) & 0xFF
    addr = cpu & 0xFFFF
    if addr < 0x8000:
        return None
    off = ((bank & 0x7F) * 0x8000) + (addr - 0x8000)
    return off if 0 <= off < size else None


def lorom_off_to_cpu(off: int) -> int:
    bank = off // 0x8000
    addr = 0x8000 + (off % 0x8000)
    return (bank << 16) | addr


def _s8(v): return v - 256 if v >= 128 else v
def _s16(v): return v - 65536 if v >= 32768 else v


def build_unique_anchors(final: bytes, beta: bytes, block=24, final_step=16):
    """Exact unique 24-byte matches. Sampling only final keeps runtime small."""
    index = {}
    duplicate = set()
    for j in range(0, len(beta) - block + 1):
        key = beta[j:j+block]
        if key in index:
            duplicate.add(key)
        else:
            index[key] = j

    pairs = []
    for i in range(0, len(final) - block + 1, final_step):
        key = final[i:i+block]
        if key in index and key not in duplicate:
            j = index[key]
            pairs.append((i, j, j-i))

    counts = Counter(delta for _,_,delta in pairs)
    # Ignore shifts supported by only a handful of anchors.
    good = {delta for delta, n in counts.items() if n >= 8}
    anchors = sorted((i,j,d) for i,j,d in pairs if d in good)
    return anchors, counts


class ShiftLookup:
    def __init__(self, anchors, counts):
        self.anchors = anchors
        self.pos = [a[0] for a in anchors]
        self.counts = counts

    def get(self, off, maxdist=1024):
        k = bisect.bisect_left(self.pos, off)
        left = self.anchors[k-1] if k > 0 else None
        right = self.anchors[k] if k < len(self.anchors) else None
        if left and right and left[2] == right[2]:
            if off-left[0] <= maxdist and right[0]-off <= maxdist:
                return left[2], "bracket", min(off-left[0], right[0]-off)
        candidates = [x for x in (left,right) if x]
        if candidates:
            near = min(candidates, key=lambda x: abs(x[0]-off))
            if abs(near[0]-off) <= 64 and self.counts[near[2]] >= 20:
                return near[2], "near", abs(near[0]-off)
        return None


def vector_seeds(data: bytes):
    out = []
    if len(data) < 0x8000:
        return out
    for voff in VECTOR_OFFSETS:
        if voff + 2 <= len(data):
            pc = int.from_bytes(data[voff:voff+2], "little")
            if pc >= 0x8000 and pc not in out:
                out.append(pc)  # vectors are bank $00
    return out


def reachable_instruction_starts(data: bytes, max_states=300000):
    """
    Conservative static control-flow walk. Tracks M/X enough to decode
    variable-width immediates. Indirect jumps terminate a path.
    """
    q = deque((pc, 1, 1) for pc in vector_seeds(data))  # reset/emulation: M=X=1
    seen = set()
    starts = set()

    while q and len(seen) < max_states:
        cpu, m, x = q.popleft()
        state = (cpu, m, x)
        if state in seen:
            continue
        seen.add(state)

        off = lorom_cpu_to_off(cpu, len(data))
        if off is None:
            continue

        op = data[off]
        spec = INSN_LEN[op]
        if spec == 'M':
            lens = [2,3] if m is None else [2 if m else 3]
        elif spec == 'X':
            lens = [2,3] if x is None else [2 if x else 3]
        else:
            lens = [spec]

        for ln in lens:
            if off + ln > len(data):
                continue
            starts.add(off)
            nm, nx = m, x

            if op == 0xC2 and ln >= 2:  # REP
                mask = data[off+1]
                if mask & 0x20: nm = 0
                if mask & 0x10: nx = 0
            elif op == 0xE2 and ln >= 2:  # SEP
                mask = data[off+1]
                if mask & 0x20: nm = 1
                if mask & 0x10: nx = 1
            elif op == 0x28:  # PLP restores status; M/X become unknown
                nm = nx = None

            bank = (cpu >> 16) & 0xFF
            pc = cpu & 0xFFFF
            fall = (bank << 16) | ((pc + ln) & 0xFFFF)

            # Path terminators.
            if op in (0x00,0x02,0x40,0x60,0x6B,0xCB,0xDB):
                continue

            # Conditional branches.
            if op in (0x10,0x30,0x50,0x70,0x90,0xB0,0xD0,0xF0):
                target = (bank << 16) | ((pc + 2 + _s8(data[off+1])) & 0xFFFF)
                q.append((target,nm,nx))
                q.append((fall,nm,nx))
                continue

            if op == 0x80:  # BRA
                target = (bank << 16) | ((pc + 2 + _s8(data[off+1])) & 0xFFFF)
                q.append((target,nm,nx))
                continue

            if op == 0x82:  # BRL
                rel = int.from_bytes(data[off+1:off+3], "little")
                target = (bank << 16) | ((pc + 3 + _s16(rel)) & 0xFFFF)
                q.append((target,nm,nx))
                continue

            if op == 0x4C:  # JMP abs
                target = (bank << 16) | int.from_bytes(data[off+1:off+3], "little")
                q.append((target,nm,nx))
                continue

            if op == 0x5C:  # JML
                target = int.from_bytes(data[off+1:off+4], "little")
                q.append((target,nm,nx))
                continue

            if op in (0x6C,0x7C,0xDC):  # indirect jumps
                continue

            if op == 0x20:  # JSR abs
                target = (bank << 16) | int.from_bytes(data[off+1:off+3], "little")
                q.append((target,nm,nx))
                q.append((fall,nm,nx))
                continue

            if op == 0x22:  # JSL
                target = int.from_bytes(data[off+1:off+4], "little")
                q.append((target,nm,nx))
                q.append((fall,nm,nx))
                continue

            q.append((fall,nm,nx))

    return starts


def context_score(final: bytes, beta: bytes, fo: int, bo: int, ln: int):
    score = 0
    for k in range(1,5):
        if fo-k >= 0 and bo-k >= 0 and final[fo-k] == beta[bo-k]:
            score += 1
    for k in range(5):
        if fo+ln+k < len(final) and bo+ln+k < len(beta) and final[fo+ln+k] == beta[bo+ln+k]:
            score += 1
    return score


def extract_evidence(final: bytes, beta: bytes, starts, shifts: ShiftLookup):
    evidence = []
    for fo in sorted(starts):
        op = final[fo]
        if op in ABS16_OPS:
            ln, kind, mnemonic = 3, "abs16", ABS16_OPS[op]
        elif op in LONG24_OPS:
            ln, kind, mnemonic = 4, "long24", LONG24_OPS[op]
        else:
            continue

        ls = shifts.get(fo)
        if not ls:
            continue
        delta_rom, align_kind, distance = ls
        bo = fo + delta_rom
        if bo < 0 or bo + ln > len(beta) or beta[bo] != op:
            continue

        score = context_score(final, beta, fo, bo, ln)
        if score < 6:
            continue

        fv = int.from_bytes(final[fo+1:fo+ln], "little")
        bv = int.from_bytes(beta[bo+1:bo+ln], "little")

        # Long operands are only useful for WRAM if they directly name $7E/$7F.
        if kind == "long24":
            fb = (fv >> 16) & 0xFF
            bb = (bv >> 16) & 0xFF
            if fb not in (0x7E,0x7F) or bb not in (0x7E,0x7F):
                continue
            foff = fv - 0x7E0000
            boff = bv - 0x7E0000
            if not (0 <= foff < 0x20000 and 0 <= boff < 0x20000):
                continue
            ram_class = "direct_long_wram"
        else:
            # Absolute addresses below $2000 are the strongest static WRAM
            # evidence because this area is the low-WRAM mirror in normal SNES banks.
            foff, boff = fv, bv
            ram_class = "low_wram_absolute" if fv < 0x2000 and bv < 0x2000 else "absolute_dbr_dependent"

        evidence.append({
            "kind": kind,
            "mnemonic": mnemonic,
            "opcode": op,
            "final_rom_offset": fo,
            "beta_rom_offset": bo,
            "final_cpu_rom": lorom_off_to_cpu(fo),
            "beta_cpu_rom": lorom_off_to_cpu(bo),
            "final_operand": fv,
            "beta_operand": bv,
            "final_wram_offset": foff,
            "beta_wram_offset": boff,
            "ram_class": ram_class,
            "operand_delta": boff - foff,
            "rom_shift": delta_rom,
            "alignment": align_kind,
            "context_score": score,
        })
    return evidence


def aggregate_mappings(evidence):
    grouped = defaultdict(list)
    for e in evidence:
        if e["ram_class"] not in ("low_wram_absolute","direct_long_wram"):
            continue
        grouped[(e["final_wram_offset"], e["beta_wram_offset"])].append(e)

    rows = []
    for (f,b), evs in grouped.items():
        rows.append({
            "final_wram_offset": f,
            "beta_wram_offset": b,
            "delta": b-f,
            "evidence_count": len(evs),
            "best_context_score": max(e["context_score"] for e in evs),
            "mnemonics": sorted(set(e["mnemonic"] for e in evs)),
            "classes": sorted(set(e["ram_class"] for e in evs)),
            "rom_sites": [e["final_rom_offset"] for e in evs],
        })
    rows.sort(key=lambda r:(r["final_wram_offset"], -r["evidence_count"]))
    return rows


def infer_rules(mapping_rows):
    """
    Infer only conservative range rules:
    - changed mappings with the same delta
    - >=10 distinct source addresses
    - no observed identity mapping inside the inferred source interval
    """
    by_delta = defaultdict(set)
    identity = set()
    for r in mapping_rows:
        f,b,d = r["final_wram_offset"], r["beta_wram_offset"], r["delta"]
        if d == 0:
            identity.add(f)
        else:
            by_delta[d].add(f)

    rules = []
    for delta, vals in by_delta.items():
        vals = sorted(vals)
        if len(vals) < 10:
            continue
        lo, hi = vals[0], vals[-1]
        conflicts = sorted(x for x in identity if lo <= x <= hi)
        if conflicts:
            continue
        rules.append({
            "start": lo,
            "end": hi,
            "delta": delta,
            "supporting_addresses": len(vals),
            "confidence": "inferred-static",
        })
    return sorted(rules, key=lambda r:(r["start"],r["end"]))


def format_ra(offset):
    return 0xF50000 + offset


def parse_query(s: str):
    t = s.strip().upper().replace("_","")
    if t.startswith("$"):
        t=t[1:]
    if ":" in t:
        bank_s, addr_s = t.split(":",1)
        bank=int(bank_s,16); addr=int(addr_s,16)
        if bank == 0x7E:
            return addr
        if bank == 0x7F:
            return 0x10000 + addr
        raise ValueError("only $7E/$7F addresses can be converted to WRAM offsets")
    if t.startswith("0X"):
        v=int(t,16)
    elif all(c in "0123456789ABCDEF" for c in t):
        v=int(t,16)
    else:
        v=int(t,0)
    if 0xF50000 <= v < 0xF70000:
        return v - 0xF50000
    if 0 <= v < 0x20000:
        return v
    raise ValueError("query is outside the supported 128 KiB WRAM range")


def translate_offset(offset, explicit, rules):
    if offset in explicit:
        return explicit[offset], "explicit"
    for r in rules:
        if r["start"] <= offset <= r["end"]:
            return offset + r["delta"], "range"
    return offset, "identity"


def write_outputs(outdir: Path, final_name, beta_name, final, beta, anchors, counts, starts, evidence, mappings, rules):
    outdir.mkdir(parents=True, exist_ok=True)

    changed = [r for r in mappings if r["delta"] != 0]
    explicit = {}
    # Only unambiguous changed pairs. If one final address has competing beta
    # targets, select none and leave it for later dynamic confirmation.
    by_src = defaultdict(list)
    for r in changed:
        by_src[r["final_wram_offset"]].append(r)
    ambiguous = []
    for src, rs in by_src.items():
        targets = {r["beta_wram_offset"] for r in rs}
        if len(targets) == 1:
            explicit[src] = next(iter(targets))
        else:
            ambiguous.append(src)

    report = {
        "final_rom": {
            "name": final_name, "size": len(final), "md5": md5(final), "title": title(final)
        },
        "beta_rom": {
            "name": beta_name, "size": len(beta), "md5": md5(beta), "title": title(beta)
        },
        "reachable_instruction_starts": len(starts),
        "anchor_count": len(anchors),
        "top_rom_shifts": [{"delta":d,"anchors":n} for d,n in counts.most_common(20)],
        "mapping_rows": len(mappings),
        "changed_mapping_rows": len(changed),
        "explicit_changed_addresses": len(explicit),
        "ambiguous_changed_sources": [f"0x{x:05X}" for x in ambiguous],
        "rules": rules,
    }
    (outdir/"summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    with (outdir/"wram_mapping.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f)
        w.writerow(["final_wram","beta_wram","delta","evidence_count","best_context","classes","mnemonics","final_RA","beta_RA"])
        for r in mappings:
            w.writerow([
                f"0x{r['final_wram_offset']:05X}",
                f"0x{r['beta_wram_offset']:05X}",
                r["delta"],
                r["evidence_count"],
                r["best_context_score"],
                "|".join(r["classes"]),
                "|".join(r["mnemonics"]),
                f"0x{format_ra(r['final_wram_offset']):06X}",
                f"0x{format_ra(r['beta_wram_offset']):06X}",
            ])

    with (outdir/"changed_only.csv").open("w",newline="",encoding="utf-8") as f:
        w=csv.writer(f)
        w.writerow(["final_wram","beta_wram","delta","evidence_count","best_context","final_RA","beta_RA"])
        for r in changed:
            w.writerow([
                f"0x{r['final_wram_offset']:05X}",
                f"0x{r['beta_wram_offset']:05X}",
                r["delta"],
                r["evidence_count"],
                r["best_context_score"],
                f"0x{format_ra(r['final_wram_offset']):06X}",
                f"0x{format_ra(r['beta_wram_offset']):06X}",
            ])

    map_json = {
        "explicit": {f"0x{k:05X}": f"0x{v:05X}" for k,v in sorted(explicit.items())},
        "range_rules": [
            {"start":f"0x{r['start']:05X}","end":f"0x{r['end']:05X}","delta":r["delta"],
             "supporting_addresses":r["supporting_addresses"],"confidence":r["confidence"]}
            for r in rules
        ],
        "note": "Static map. Explicit entries come from matched reachable 65C816 RAM accesses; range rules are inferred where there were no observed identity conflicts."
    }
    (outdir/"address_map.json").write_text(json.dumps(map_json, indent=2),encoding="utf-8")

    module = [
        '"""Generated Tom & Jerry final -> BETA WRAM translation."""',
        "",
        "EXPLICIT = {",
    ]
    for k,v in sorted(explicit.items()):
        module.append(f"    0x{k:05X}: 0x{v:05X},")
    module += ["}", "", "RANGE_RULES = ["]
    for r in rules:
        module.append(f"    (0x{r['start']:05X}, 0x{r['end']:05X}, {r['delta']}),")
    module += [
        "]", "",
        "def translate_wram_offset(offset: int) -> int:",
        "    offset = int(offset)",
        "    if offset in EXPLICIT:",
        "        return EXPLICIT[offset]",
        "    for start, end, delta in RANGE_RULES:",
        "        if start <= offset <= end:",
        "            return offset + delta",
        "    return offset",
        "",
        "def translate_ra_address(address: int) -> int:",
        "    address = int(address)",
        "    if 0xF50000 <= address < 0xF70000:",
        "        return 0xF50000 + translate_wram_offset(address - 0xF50000)",
        "    return address",
        "",
    ]
    (outdir/"tomjerry_address_map.py").write_text("\n".join(module),encoding="utf-8")

    lines = []
    lines.append("TOM & JERRY FINAL -> BETA STATIC AUTO-MAP")
    lines.append("="*48)
    lines.append(f"Final: {final_name}")
    lines.append(f"  MD5: {md5(final)}")
    lines.append(f"BETA : {beta_name}")
    lines.append(f"  MD5: {md5(beta)}")
    lines.append("")
    lines.append(f"Reachable 65C816 instruction starts: {len(starts)}")
    lines.append(f"Exact ROM alignment anchors: {len(anchors)}")
    lines.append("Most common ROM shifts (BETA offset - final offset):")
    for d,n in counts.most_common(10):
        lines.append(f"  {d:+6d} (0x{(d & 0xffffffff):X}) : {n} anchors")
    lines.append("")
    lines.append(f"Unambiguous changed WRAM addresses: {len(explicit)}")
    if rules:
        lines.append("Inferred range rules:")
        for r in rules:
            lines.append(f"  0x{r['start']:05X}-0x{r['end']:05X} -> +{r['delta']}  ({r['supporting_addresses']} observed source addresses)")
    else:
        lines.append("No conservative range rule was inferred.")
    lines.append("")
    lines.append("The bridge should translate after converting the RA virtual address to a WRAM offset.")
    lines.append("Use translate_wram_offset(offset), or translate_ra_address(address) if the bridge still has the F5xxxx RA address.")
    (outdir/"REPORT.txt").write_text("\n".join(lines)+"\n",encoding="utf-8")

    return report, explicit


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--final", required=True, help="Supported final ROM")
    ap.add_argument("--beta", required=True, help="BETA/prototype ROM")
    ap.add_argument("--out", default="tomjerry_auto_map_out")
    ap.add_argument("--query", action="append", default=[], help="Optional WRAM/RA address: F50EA6, 0x0EA6, $7E:0EA6")
    args=ap.parse_args()

    raw_final=Path(args.final).read_bytes()
    raw_beta=Path(args.beta).read_bytes()
    final,hf=strip_copier_header(raw_final)
    beta,hb=strip_copier_header(raw_beta)

    if len(final) != len(beta):
        print(f"Warning: ROM sizes differ after header stripping: {len(final)} vs {len(beta)}")

    anchors,counts=build_unique_anchors(final,beta)
    shifts=ShiftLookup(anchors,counts)
    starts=reachable_instruction_starts(final)
    evidence=extract_evidence(final,beta,starts,shifts)
    mappings=aggregate_mappings(evidence)
    rules=infer_rules(mappings)
    report,explicit=write_outputs(Path(args.out),Path(args.final).name,Path(args.beta).name,final,beta,anchors,counts,starts,evidence,mappings,rules)

    print(f"Final MD5 : {report['final_rom']['md5']}")
    print(f"BETA MD5  : {report['beta_rom']['md5']}")
    print(f"Anchors    : {report['anchor_count']}")
    print(f"Code starts: {report['reachable_instruction_starts']}")
    print(f"Changed explicit WRAM mappings: {report['explicit_changed_addresses']}")
    for r in rules:
        print(f"Rule: 0x{r['start']:05X}-0x{r['end']:05X} => offset {r['delta']:+d} ({r['supporting_addresses']} addresses support it)")

    if args.query:
        print("")
        for q in args.query:
            try:
                off=parse_query(q)
                new,how=translate_offset(off,explicit,rules)
                print(f"{q}: WRAM 0x{off:05X} -> 0x{new:05X} [{how}] | RA 0x{format_ra(off):06X} -> 0x{format_ra(new):06X}")
            except Exception as e:
                print(f"{q}: ERROR: {e}")

if __name__ == "__main__":
    main()
