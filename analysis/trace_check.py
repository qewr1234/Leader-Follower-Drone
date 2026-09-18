"""
analysis/trace_check.py — docs/REQUIREMENTS.md 의 추적성 표가 실제 코드·시험과 맞는지 검사

    python3 analysis/trace_check.py            # 깨진 참조가 있으면 종료코드 1

요구도 표의 '검증 근거' 칸에 쓰는 토큰:
  UT[접두어]      test_fixes.py 의 check("...") 이름이 그 접두어로 시작하는 검사가 1개 이상
  CL[접두어]      test_closed_loop.py 의 check("...") 이름이 그 접두어로 시작하는 검사가 1개 이상
  SITL[이름]      sitl/harness.py 의 시나리오 이름
  AN[경로]        docs/stability_margins.json 의 키 경로 (예: axes.forward.gm_db)
  INSPECT[파일:문자열]  그 파일에 그 문자열이 그대로 있음
  HW              하드웨어 보고 (저장소에 근거 없음, VERIFICATION.md 참고)
  —               근거 없음 (미검증)
"""

import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REQ_MD = os.path.join(ROOT, "docs", "REQUIREMENTS.md")
TOKEN = re.compile(r"(UT|CL|SITL|AN|INSPECT)\[([^\]]+)\]|\b(HW)\b")
ROW = re.compile(r"^\|\s*((?:FCR|EST|SAF|IF|OPS)-\d+)\s*\|")


def _checks(path):
    src = open(os.path.join(ROOT, path), encoding="utf-8").read()
    return re.findall(r'check\(\s*"((?:[^"\\]|\\.)*)"', src)


def _scenarios():
    src = open(os.path.join(ROOT, "sitl", "harness.py"), encoding="utf-8").read()
    return set(re.findall(r'\b(?:el)?if name == "([a-z_0-9]+)"', src))


def _json_has(data, dotted):
    """키 이름에 점이 들어갈 수 있으므로(kff0.8, scale0.55) 앞에서부터 가능한 모든 분할을 시도한다."""
    parts = dotted.split(".")

    def walk(cur, i):
        if i == len(parts):
            return True
        for j in range(i + 1, len(parts) + 1):
            key = ".".join(parts[i:j])
            if isinstance(cur, dict) and key in cur and walk(cur[key], j):
                return True
            if isinstance(cur, list) and key.isdigit() and int(key) < len(cur) and walk(cur[int(key)], j):
                return True
        return False
    return walk(data, 0)


def load_requirements(path=REQ_MD):
    rows = []
    for line in open(path, encoding="utf-8"):
        m = ROW.match(line)
        if not m:
            continue
        cells = [c.strip() for c in re.split(r"(?<!\\)\|", line.strip().strip("|"))]
        if len(cells) < 6:          # 3절 요약표(ID | 상태 | 필요한 것) 는 요구도 정의가 아니다
            continue
        rows.append({"id": m.group(1), "cells": cells, "line": line})
    return rows


def run(verbose=True):
    ut = _checks("test_fixes.py"); cl = _checks("test_closed_loop.py"); scen = _scenarios()
    an = None
    an_path = os.path.join(ROOT, "docs", "stability_margins.json")
    if os.path.exists(an_path):
        an = json.load(open(an_path, encoding="utf-8"))
    rows = load_requirements()
    errors, used_ut, used_cl, used_scen = [], set(), set(), set()
    ids = [r["id"] for r in rows]
    dup = {i for i in ids if ids.count(i) > 1}
    if dup:
        errors.append(f"중복 ID: {sorted(dup)}")
    for r in rows:
        evidence = r["cells"][-2] if len(r["cells"]) >= 2 else ""
        toks = TOKEN.findall(evidence)
        if not toks and "—" not in evidence:
            errors.append(f"{r['id']}: 검증 근거 칸에 토큰이 없음: {evidence!r}")
        for kind, arg, hw in toks:
            if hw:
                continue
            arg = arg.strip()
            if kind == "UT":
                hits = [c for c in ut if c.startswith(arg)]
                if not hits:
                    errors.append(f"{r['id']}: test_fixes.py 에 '{arg}' 로 시작하는 check 없음")
                used_ut.update(hits)
            elif kind == "CL":
                hits = [c for c in cl if c.startswith(arg)]
                if not hits:
                    errors.append(f"{r['id']}: test_closed_loop.py 에 '{arg}' 로 시작하는 check 없음")
                used_cl.update(hits)
            elif kind == "SITL":
                if arg not in scen:
                    errors.append(f"{r['id']}: sitl/harness.py 에 시나리오 '{arg}' 없음 (있는 것: {sorted(scen)})")
                used_scen.add(arg)
            elif kind == "AN":
                if an is None:
                    errors.append(f"{r['id']}: docs/stability_margins.json 이 없음 (analysis/stability_margins.py 실행)")
                elif not _json_has(an, arg):
                    errors.append(f"{r['id']}: stability_margins.json 에 키 '{arg}' 없음")
            elif kind == "INSPECT":
                fpath, _, needle = arg.partition(":")
                full = os.path.join(ROOT, fpath.strip())
                if not os.path.exists(full) or needle.strip() not in open(full, encoding="utf-8").read():
                    errors.append(f"{r['id']}: {fpath} 에 문자열 {needle.strip()!r} 없음")
    summary = {"requirements": len(rows), "errors": errors,
               "ut_total": len(set(ut)), "ut_traced": len(used_ut), "ut_orphans": sorted(set(ut) - used_ut),
               "cl_total": len(set(cl)), "cl_traced": len(used_cl), "cl_orphans": sorted(set(cl) - used_cl),
               "sitl_total": len(scen), "sitl_traced": len(used_scen & scen), "sitl_orphans": sorted(scen - used_scen)}
    if verbose:
        print(f"요구도 {summary['requirements']}개, 깨진 참조 {len(errors)}개")
        for e in errors:
            print("  ✗", e)
        print(f"역추적: test_fixes {summary['ut_traced']}/{summary['ut_total']}, "
              f"test_closed_loop {summary['cl_traced']}/{summary['cl_total']}, SITL {summary['sitl_traced']}/{summary['sitl_total']}")
        for k in ("ut_orphans", "cl_orphans", "sitl_orphans"):
            if summary[k]:
                print(f"  요구도에 안 묶인 {k}: {len(summary[k])}개")
                for o in summary[k]:
                    print("    -", o)
    return summary


if __name__ == "__main__":
    s = run()
    sys.exit(1 if s["errors"] else 0)
