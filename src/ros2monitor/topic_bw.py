#!/usr/bin/env python

import time
from rclpy.time import Time

from ros2topic.verb.bw import ROSTopicBandwidth


class TopicBandwidth(ROSTopicBandwidth):

    def __init__(self, node, window_size):
        super().__init__(node, window_size)

    def get_bw(self):
        """Get the average publishing bw."""
        if len(self.times) < 2:
            return None, None, None, None, None
        with self.lock:
            n = len(self.times)
            tn = time.monotonic()
            t0 = self.times[0]

            if isinstance(t0, Time):
                t0 = t0.nanoseconds / 1e9

            total = sum(self.sizes)
            bytes_per_s = total / (tn - t0) if (tn - t0) > 0 else 0
            mean = total / n

            # min and max
            max_s = max(self.sizes)
            min_s = min(self.sizes)

        return bytes_per_s, n, mean, min_s, max_s

    def get_bw_text(self):
        bandwidth_text = "unknown"
        bytes_per_s, _, _, _, _ = self.get_bw()
        if bytes_per_s is None:
            bandwidth_text = "unknown"
        elif bytes_per_s < 1000:
            bandwidth_text = "%.2fB/s" % bytes_per_s
        elif bytes_per_s < 1000000:
            bandwidth_text = "%.2fKB/s" % (bytes_per_s / 1000.0)
        else:
            bandwidth_text = "%.2fMB/s" % (bytes_per_s / 1000000.0)
        return bandwidth_text
