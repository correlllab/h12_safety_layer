'''Minimal raw DDS estop subscriber'''

import threading
import time
from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.domain import DomainParticipant


@dataclass
class EStopStatus(IdlStruct, typename='h12.estop.EstopStatusRaw'):  # type: ignore[misc, call-arg]
    triggered: bool
    plugged_in: bool
    sequence_id: int
    stamp_ms: int


class EStopSubscriber:
    '''Subscribe to raw estop DDS topic and cache latest sample'''
    def __init__(
        self, topic: str = 'h12/estop_status_raw', poll_hz: float = 500.0
    ) -> None:
        self._topic_name = topic
        self._poll_hz = float(poll_hz)
        if self._poll_hz <= 0.0:
            raise ValueError('poll_hz must be positive')
        self._poll_dt = 1.0 / self._poll_hz
        self._lock = threading.Lock()
        self._running = True
        self._latest: EStopStatus | None = None
        self._last_error: str | None = None

        self._participant = None
        self._subscriber = None
        self._topic = None
        self._reader = None

        # initialize reader entities before thread start
        self._init_reader()

        # start eager background subscription loop
        self._thread = threading.Thread(
            target=self._read_loop, name='estop_subscriber', daemon=True
        )
        self._thread.start()

    def _init_reader(self) -> None:
        '''Initialize CycloneDDS reader objects'''
        # create participant, subscriber, topic, and reader once
        self._participant = DomainParticipant()
        self._subscriber = Subscriber(self._participant)
        self._topic = Topic(self._participant, self._topic_name, EStopStatus)
        self._reader = DataReader(self._subscriber, self._topic)

    def _read_loop(self) -> None:
        '''Continuously read and cache latest estop sample'''
        reader = self._reader
        if reader is None:
            with self._lock:
                self._last_error = 'reader not initialized'
            return

        while self._running:
            try:
                samples = reader.take()
                if not samples:
                    # avoid busy wait when no sample is available
                    time.sleep(self._poll_dt)
                    continue

                # keep latest sample only for simple property access
                sample = samples[-1]
                latest = EStopStatus(
                    triggered=bool(sample.triggered),
                    plugged_in=bool(sample.plugged_in),
                    sequence_id=int(sample.sequence_id),
                    stamp_ms=int(sample.stamp_ms),
                )
                with self._lock:
                    self._latest = latest
            except Exception as exc:
                # record error and back off briefly
                with self._lock:
                    self._last_error = str(exc)
                time.sleep(self._poll_dt)

    @property
    def latest(self) -> EStopStatus | None:
        '''Return latest estop sample or None'''
        with self._lock:
            return self._latest

    @property
    def triggered(self) -> bool:
        '''Return latest estop triggered state'''
        with self._lock:
            if self._latest is None:
                return False
            return self._latest.triggered

    @property
    def plugged_in(self) -> bool:
        '''Return latest estop plugged-in state'''
        with self._lock:
            if self._latest is None:
                return False
            return self._latest.plugged_in

    @property
    def last_error(self) -> str | None:
        '''Return last subscriber read error'''
        with self._lock:
            return self._last_error

    def close(self) -> None:
        '''Stop background reader thread'''
        self._running = False
        self._thread.join(timeout=1.0)
