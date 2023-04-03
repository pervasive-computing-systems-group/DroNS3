import abc
import time

class Telemetry(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        pass

    def __str__(self) -> str:
        pass


class WSNData(Telemetry):
    def __init__(self):
        super().__init__()
        self.start_time = time.time()
        self.data_collected = 0
        self.total_time = None

    def start_timer(self):
        self.start_time = time.time()
    
    def stop_timer(self):
        self.total_time = time.time() - self.start_time
    
    def collect_data(self, data):
        self.data_collected += data
        
    def __str__(self) -> str:
        return "{t},{d}".format(t = self.total_time if self.total_time is not None else time.time() - self.start_time, d = self.data_collected)

class NetworkData(Telemetry):

    def __init__(self):
        super().__init__()
        self.data_collected = 0
    
    def add_data_collected(self, data:int):
        self.data_collected += data

    def __str__(self) -> str:
        return "{d}".format(d = self.data_collected)
