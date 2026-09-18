"""
logger.py — 실험 로깅 (JSONL 스트리밍 + 종료 시 CSV 변환)

실행 중에는 row마다 flush되는 JSONL로 기록하고(크래시에도 데이터 보존),
close() 시 전체 키의 합집합으로 CSV를 생성한다. csv.DictWriter를 바로 쓰면
첫 row에서 헤더가 고정되어 뒤늦게 등장하는 키(예: 첫 탐지 시 track.*)가
ValueError로 메인 루프를 죽이기 때문이다.
"""

import csv
import json
import os
import time

import numpy as np

_PLAIN = (int, float, str, bool, type(None))


def _to_scalar(v):
    if isinstance(v, np.ndarray):
        return ";".join(f"{float(x):.6g}" for x in v.ravel())
    if isinstance(v, np.floating):
        return float(v)
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.bool_):
        return bool(v)
    return v


def _flatten(row, prefix, out):
    """중첩 dict 를 'a.b.c' 키로 편다. 매 프레임 100개 넘는 값을 다루므로 평범한 스칼라는 함수 호출 없이 통과."""
    for k, v in row.items():
        key = f"{prefix}.{k}" if prefix else k
        if type(v) in _PLAIN:
            out[key] = v
        elif isinstance(v, dict):
            _flatten(v, key, out)
        elif isinstance(v, (list, tuple)):
            out[key] = ";".join(str(_to_scalar(x)) for x in v)
        else:
            out[key] = _to_scalar(v)
    return out


class ExperimentLogger:
    def __init__(self, log_dir="logs", prefix="mars_imm"):
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.jsonl_path = os.path.join(log_dir, f"{prefix}_{ts}.jsonl")
        self.csv_path = os.path.join(log_dir, f"{prefix}_{ts}.csv")
        self.file = open(self.jsonl_path, "w")
        print(f"[LOG] writing: {self.jsonl_path} (CSV는 종료 시 생성)")

    def log(self, row):
        try:
            self.file.write(json.dumps(self._flatten(row), ensure_ascii=False, default=str) + "\n")
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
            with open(self.jsonl_path) as f:
                rows = [json.loads(line) for line in f if line.strip()]
            if not rows:
                return
            fieldnames = list(dict.fromkeys(k for r in rows for k in r))   # 등장 순서 유지 합집합
            with open(self.csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames, restval="")
                writer.writeheader()
                writer.writerows(rows)
            print(f"[LOG] CSV written: {self.csv_path} ({len(rows)} rows)")
        except Exception as exc:
            print(f"[LOG][WARN] CSV convert failed ({exc}); JSONL은 보존됨: {self.jsonl_path}")

    @staticmethod
    def _flatten(row, prefix=""):
        return _flatten(row, prefix, {})
