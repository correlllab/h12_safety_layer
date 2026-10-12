'''Listener-free DDS reader that returns only the newest sample'''

from typing import Any

from cyclonedds.domain import DomainParticipant
from cyclonedds.internal import InvalidSample
from cyclonedds.qos import Policy, Qos
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic


# KEEP_LAST(1) lets CycloneDDS's C receive thread overwrite the cached sample,
# so a consumer that falls behind skips stale samples instead of replaying
# them late. There is deliberately no listener: a Python callback runs on the
# receive thread and must take the GIL, and a receive thread waiting on the
# GIL leaves packets queued in the socket buffer, which turns into a standing
# command delay of seconds.
LATEST_ONLY_QOS = Qos(
    Policy.History.KeepLast(1),
    Policy.Reliability.BestEffort,
)
# Upper bound on samples drained per take; with KEEP_LAST(1) at most one is
# ever cached, so this only guards against a QoS mismatch.
TAKE_BATCH = 8


class LatestReader:
    '''Poll the newest sample of a DDS topic without a receive-thread callback'''
    def __init__(
        self, participant: DomainParticipant, topic_name: str, msg_type: Any
    ) -> None:
        self._topic = Topic(participant, topic_name, msg_type)
        self._reader = DataReader(participant, self._topic, LATEST_ONLY_QOS)

    def take_latest(self) -> Any | None:
        '''Return the newest valid sample not yet taken, or None'''
        latest = None
        for sample in self._reader.take(N=TAKE_BATCH):
            if not isinstance(sample, InvalidSample):
                latest = sample
        return latest

    def close(self) -> None:
        '''Release the reader'''
        self._reader = None
        self._topic = None
