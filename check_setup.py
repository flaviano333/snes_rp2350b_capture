#!/usr/bin/env python3
import argparse
import hashlib
import importlib.util
from pathlib import Path

EXPECTED_FINAL_MD5 = "bc16be2e9c7e170f7cd10da919f3e099"


def cartridge_bytes(path: Path) -> bytes:
    data = path.read_bytes()
    return data[512:] if (len(data) % 1024) == 512 else data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rom", nargs="?", default="Tom and Jerry (USA).sfc")
    args = ap.parse_args()

    print("=== DEPENDENCIAS ===")
    ok = True
    for mod in ("serial", "websockets"):
        found = importlib.util.find_spec(mod) is not None
        print(f"{mod:10s}: {'OK' if found else 'FALTANDO'}")
        ok &= found

    print("\n=== AUTO-MAP ===")
    try:
        from tomjerry_address_map import translate_ra_address, EXPLICIT, RANGE_RULES
        print(f"modulo     : OK ({len(EXPLICIT)} referencias explicitas)")
        for start, end, delta in RANGE_RULES:
            print(f"regra      : {start:05X}-{end:05X} -> delta {delta:+d}")
        for addr in (0xF51564, 0xF5287A):
            print(f"teste      : {addr:06X} -> {translate_ra_address(addr):06X}")
    except Exception as e:
        print(f"modulo     : ERRO: {e}")
        ok = False

    print("\n=== ROM FINAL ===")
    p = Path(args.rom).expanduser()
    if not p.is_file():
        print(f"arquivo    : NAO ENCONTRADO ({p})")
        print("Coloque 'Tom and Jerry (USA).sfc' nesta pasta ou passe o caminho como argumento.")
        ok = False
    else:
        body = cartridge_bytes(p)
        md5 = hashlib.md5(body).hexdigest()
        print(f"arquivo    : {p}")
        print(f"tamanho    : {len(body)} bytes")
        print(f"MD5        : {md5}")
        print(f"esperado   : {EXPECTED_FINAL_MD5}")
        if md5 != EXPECTED_FINAL_MD5:
            print("resultado  : ROM diferente da USA final esperada")
            ok = False
        else:
            print("resultado  : OK")

    print("\nSETUP:", "OK" if ok else "VERIFIQUE OS ITENS ACIMA")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
