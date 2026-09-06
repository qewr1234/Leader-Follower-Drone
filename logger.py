"""
logger.py — 실험 로깅 (JSONL 스트리밍 + 종료 시 CSV 변환)

변경점:
- 기존 csv.DictWriter는 첫 row에서 헤더가 고정되어, 이후 새 키가 생기면
  (예: 첫 탐지 시점에 track.* 필드 등장) ValueError로 메인 루프 전체가 죽었다.
- 실행 중에는 row마다 flush되는 JSONL로 기록하고(크래시에도 데이터 보존),
  close() 시 전체 키의 합집합으로 CSV를 생성한다.
"""

import csv
import json
import os
import time

import numpy as np


def _to_scalar(v):
    if isinstance(v, np.ndarray):
        return ";".join(f"{float(x):.6g}" for x in v.ravel())
    if isinstance(v, (np.floating,)):
        return float(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.bool_,)):
        return bool(v)
    return v


class ExperimentLogger:
    def __init__(self, log_dir="logs", prefix="mars_imm"):
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.jsonl_path = os.path.join(log_dir, f"{prefix}_{ts}.jsonl")
        self.csv_path = os.path.join(log_dir, f"{prefix}_{ts}.csv")
        self.path = self.csv_path
        self.file = open(self.jsonl_path, "w")
        print(f"[LOG] writing: {self.jsonl_path} (CSV는 종료 시 생성)")

    def log(self, row):
        flat = self._flatten(row)
        try:
            self.file.write(json.dumps(flat, ensure_ascii=False, default=str) + "\n")
            self.file.flush()
        except Exception as exc:
            # 로깅 실패가 제어 루프를 죽여서는 안 된다
            print(f"[LOG][WARN] row skipped: {exc}")

    def close(self):
        if self.file is None:
            return
        self.file.close()
        self.file = None
        self._write_csv()

    def _write_csv(self):
        try:
            rows = []
            with open(self.jsonl_path) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        rows.append(json.loads(line))

            if not rows:
                return

            # 등장 순서를 유지한 키 합집합
            fieldnames = []
            seen = set()
            for r in rows:
                for k in r.keys():
                    if k not in seen:
                        seen.add(k)
                        fieldnames.append(k)

            with open(self.csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames, restval="")
                writer.writeheader()
                writer.writerows(rows)

            print(f"[LOG] CSV written: {self.csv_path} ({len(rows)} rows)")

        except Exception as exc:
            print(f"[LOG][WARN] CSV convert failed ({exc}); JSONL은 보존됨: {self.jsonl_path}")

    def _flatten(self, row, prefix=""):
        out = {}
        for k, v in row.items():
            key = f"{prefix}{k}" if not prefix else f"{prefix}.{k}"
            if isinstance(v, dict):
                out.update(self._flatten(v, key))
            elif isinstance(v, (list, tuple)):
                out[key] = ";".join(str(_to_scalar(x)) for x in v)
            else:
                out[key] = _to_scalar(v)
        return out
