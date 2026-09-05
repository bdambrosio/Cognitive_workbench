"""Per-client request throttle. A limit of zero means no throttle."""
import time
from collections import defaultdict, deque


class Throttle:
    def __init__(self, per_minute: int):
        self.per_minute = per_minute
        self._seen = defaultdict(deque)

    def check(self, request):
        if self.per_minute <= 0:
            return None
        now = time.time()
        q = self._seen[request.remote_addr]
        while q and q[0] < now - 60:
            q.popleft()
        if len(q) >= self.per_minute:
            return 429, "slow down"
        q.append(now)
        return None
