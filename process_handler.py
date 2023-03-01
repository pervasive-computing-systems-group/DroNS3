import subprocess as sb
import os
import signal

class ProcessHandler():

    def __init__(self, debug = False):

        self.index = -1
        self.processes = {}
        self.ended = []
        self.debug = debug

    def add_process(self, process) -> int:
        self.index += 1
        self.processes[self.index] = process
        if self.debug:
            print("Added process {i} to list.".format(i = self.index))
        return self.index

    def remove_process(self, index):
        self.ended.append(index)
        if self.debug:
            print("Removed process {i}.".format(i = index))
    
    def end_processes(self):

        for index in range(self.index + 1):
            try:
                if index not in self.ended:
                    self.processes[index].terminate()
                    if self.debug:
                        print("Terminated process {i}".format(i = index))
            except:
                if self.debug:
                    print("Process {i} failed to terminate. The process likely ended.".format(i = index))
            
        
