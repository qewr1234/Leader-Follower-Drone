"""
logger.py — 실험 로깅 (JSONL 스트리밍 + 종료 시 CSV 변환)

실행 중에는 row마다 flush되는 JSONL로 기록하고(크래시에도 데이터 보존, 1초마다 fsync 로 전원 차단에도 최근 1초 안쪽까지 보존),
파일이 max_mb 를 넘으면 _part2, _part3 … 로 돌린다(로테이션 — 긴 비행에서 디스크를 다 쓰지 않게).
close() 시 전체 키의 합집합으로 CSV를 생성한다. csv.DictWriter를 바로 쓰면
첫 row에서 헤더가 고정되어 뒤늦게 등장하는 키(예: 첫 탐지 시 track.*)가
ValueError로 메인 루프를 죽이기 때문이다. CSV 변환은 두 번 훑는 스트리밍이라(1차 키 합집합, 2차 행 기록) 전체를 메모리에 올리지 않는다.
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
    def __init__(self, log_dir="logs", prefix="mars_imm", max_mb=200.0, fsync_sec=1.0):
        os.makedirs(log_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._base = os.path.join(log_dir, f"{prefix}_{ts}")
        self.max_bytes = float(max_mb) * 1024 * 1024 if max_mb else None
        self.fsync_sec = float(fsync_sec)
        self.part = 1
        self.paths = []                      # 돌린 JSONL 파일 전부 (CSV 는 합쳐서 하나)
        self.file = None
        self._open_part()
        self.csv_path = self._base + ".csv"
        self._last_fsync = time.monotonic()
        print(f"[LOG] writing: {self.jsonl_path} (CSV는 종료 시 생성, {max_mb} MB 마다 로테이션)")

    def _open_part(self):
        if self.file is not None:
            self.file.close()
        self.jsonl_path = self._base + (".jsonl" if self.part == 1 else f"_part{self.part}.jsonl")
        self.file = open(self.jsonl_path, "w")
        self.paths.append(self.jsonl_path)

    def log(self, row):
        try:
            self.file.write(json.dumps(self._flatten(row), ensure_ascii=False, default=str) + "\n")
            self.file.flush()
            now = time.monotonic()
            if now - self._last_fsync >= self.fsync_sec:
                os.fsync(self.file.fileno())
                self._last_fsync = now
            if self.max_bytes and self.file.tell() >= self.max_bytes:
                self.part += 1
                self._open_part()
                print(f"[LOG] rotate → {self.jsonl_path}")
        except Exception as exc:
            # 로깅 실패가 제어 루프를 죽여서는 안 된다
            print(f"[LOG][WARN] row skipped: {exc}")

    def close(self):
        if self.file is None:
            return
        try:
            os.fsync(self.file.fileno())
        except Exception:
            pass
        self.file.close()
        self.file = None
        self._write_csv()

    def _iter_rows(self):
        for path in self.paths:
            with open(path) as f:
                for line in f:
                    if line.strip():
                        try:
                            yield json.loads(line)
                        except json.JSONDecodeError:
                            continue          # 전원 차단으로 잘린 마지막 줄

    def _write_csv(self):
        try:
            fieldnames = {}
            n = 0
            for r in self._iter_rows():             # 1차: 등장 순서 유지 키 합집합
                n += 1
                for k in r:
                    fieldnames.setdefault(k, None)
            if n == 0:
                return
            with open(self.csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(fieldnames), restval="")
                writer.writeheader()
                for r in self._iter_rows():         # 2차: 행 스트리밍
                    writer.writerow(r)
            print(f"[LOG] CSV written: {self.csv_path} ({n} rows, {len(self.paths)} file(s))")
        except Exception as exc:
            print(f"[LOG][WARN] CSV convert failed ({exc}); JSONL은 보존됨: {self.paths}")

    @staticmethod
    def _flatten(row, prefix=""):
        return _flatten(row, prefix, {})
