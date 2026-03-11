'''Chunked npz logger for high-rate numeric telemetry'''
import queue
import threading
import numpy as np
from typing import Any
from pathlib import Path


class ChunkLogger:
    '''Write telemetry samples to chunked npz files in a background thread'''
    _keys = (
        'time_stamp',
        'mode',
        'q',
        'dq',
        'ddq',
        'tau_est',
        'temperature',
        'vol',
        'sensor',
        'motorstate',
        'reserve',
        'imu_quaternion',
        'imu_gyroscope',
        'imu_accelerometer',
        'imu_rpy',
        'imu_temperature',
        'q_cmd',
        'dq_cmd',
        'tau_cmd',
    )

    def __init__(
        self,
        base_dir: str,
        chunk_prefix: str,
        samples_per_chunk: int,
        write_hz: float,
        max_queue_size: int,
    ):
        self._base_dir = Path(base_dir)
        self._base_dir.mkdir(parents=True, exist_ok=True)
        self._chunk_prefix = chunk_prefix
        self._samples_per_chunk = max(1, samples_per_chunk)
        self._write_interval = max(1e-3, 1.0 / write_hz)
        self._queue: queue.Queue[dict[str, Any] | None] = queue.Queue(maxsize=max_queue_size)
        self._thread = threading.Thread(target=self._worker, name='npz_logger', daemon=True)
        self._running = False
        self._chunk_idx = self._next_chunk_index()
        self._buffers: dict[str, list[np.ndarray]] = {k: [] for k in self._keys}

    def start(self) -> None:
        '''Start logger thread'''
        self._running = True
        self._thread.start()

    def log_sample(self, sample: dict[str, Any]) -> None:
        '''Queue one sample'''
        try:
            self._queue.put_nowait(sample)
        except queue.Full:
            try:
                _ = self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(sample)
            except queue.Full:
                return

    def stop(self) -> None:
        '''Stop logger and flush pending samples'''
        if not self._running:
            return
        self._running = False
        self._queue.put(None)
        self._thread.join(timeout=3.0)

    def _worker(self) -> None:
        while True:
            try:
                item = self._queue.get(timeout=self._write_interval)
            except queue.Empty:
                self._flush_if_needed(force=False)
                continue

            if item is None:
                self._drain()
                self._flush_if_needed(force=True)
                return

            self._append(item)
            self._flush_if_needed(force=False)

    def _append(self, sample: dict[str, Any]) -> None:
        for key in self._keys:
            if key not in sample:
                raise KeyError(f'missing key in sample: {key}')
            self._buffers[key].append(np.asarray(sample[key]))

    def _flush_if_needed(self, force: bool) -> None:
        size = len(self._buffers['time_stamp'])
        if size == 0:
            return
        if not force and size < self._samples_per_chunk:
            return

        arrays: dict[str, np.ndarray] = {}
        for key in self._keys:
            if key == 'time_stamp':
                arrays[key] = np.asarray(self._buffers[key], dtype=np.float64)
            elif key == 'imu_temperature':
                arrays[key] = np.asarray(self._buffers[key], dtype=np.int16)
            else:
                arrays[key] = np.stack(self._buffers[key], axis=0)

        out_path = self._base_dir / f'{self._chunk_prefix}{self._chunk_idx:06d}.npz'
        np.savez(out_path, **arrays)
        self._chunk_idx += 1
        self._buffers = {k: [] for k in self._keys}

    def _drain(self) -> None:
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                return
            if item is not None:
                self._append(item)

    def _next_chunk_index(self) -> int:
        pattern = f'{self._chunk_prefix}*.npz'
        existing = sorted(self._base_dir.glob(pattern))
        if not existing:
            return 0
        last = existing[-1].stem
        suffix = last.removeprefix(self._chunk_prefix)
        try:
            return int(suffix) + 1
        except ValueError:
            return len(existing)
