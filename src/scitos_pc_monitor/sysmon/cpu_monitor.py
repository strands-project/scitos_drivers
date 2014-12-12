# encoding=utf-8
from status_monitor import  StatusMonitor
import re
import subprocess

class CPUTemperature(StatusMonitor):
    def __init__(self):
        self._freq=0.2
        cpuinfo = subprocess.check_output(['cat','/proc/cpuinfo'])
        self._number_cores = int(re.search("cpu cores\s+:\s+(\d+)",cpuinfo).group(1))
#        usage = subprocess.check_output(['mpstat','-P','ALL']).decode('utf-8')
#        self._number_threads = int(re.search("\((\d+) CPU\)",usage).group(1))

        super(CPUTemperature, self).__init__(self._freq)

        self._temps_re = re.compile(ur"Core (\d):\s+\+(\d+)", re.UNICODE)
#        self._usage_re = re.compile(ur"\d+:\d+:\d+\s+(\d+)(\s+\d+\.\d+){9}", re.UNICODE)

        self._name="CPU Temperature"

    def get_fields(self):
        speeds = ["cpu%d_mhz"%d for d in range(self._number_cores) ]
        temps = ["cpu%d_temp"%d for d in range(self._number_cores) ]
#        usage = ["cpu%d_usage"%d for d in range(self._number_threads) ]

        return speeds + temps# + usage
        
    def update(self):
        cpuinfo = subprocess.check_output(['cat','/proc/cpuinfo'])
        for i in range(self._number_cores):
            m = re.search('MHz\s+:\s+(\d+).*\n.*\n.*\n.*\ncore id\s+:\s+%d'%i, cpuinfo)
            setattr(self,"_cpu%d_mhz"%i,m.group(1))
        
        sensors = subprocess.check_output(['sensors']).decode('utf-8')
        for m in self._temps_re.finditer(sensors):
            setattr(self,"_cpu%s_temp"%m.group(1),int( m.group(2)) )
        
#        # need systat package
#        usage = subprocess.check_output(['mpstat','-P','ALL']).decode('utf-8')
#        for m  in self._usage_re.finditer(usage):
#            setattr(self,"_cpu%s_usage" % m.group(1), 100.0 - float(m.group(2).strip()))

            
class CPUUsage(StatusMonitor):
    def __init__(self):
        self._freq=0.2

        usage = subprocess.check_output(['mpstat','-P','ALL']).decode('utf-8')
        self._number_threads = int(re.search("\((\d+) CPU\)",usage).group(1))

        super(CPUUsage, self).__init__(self._freq)

        self._usage_re = re.compile(ur"\d+:\d+:\d+\s+(\d+)(\s+\d+\.\d+){9}", re.UNICODE)

        self._name="CPU Usage"

    def get_fields(self):
        usage = ["cpu%d_usage"%d for d in range(self._number_threads) ]
        return usage
        
    def update(self):
        # need systat package
        usage = subprocess.check_output(['mpstat','-P','ALL']).decode('utf-8')
        for m  in self._usage_re.finditer(usage):
            setattr(self,"_cpu%s_usage" % m.group(1), 100.0 - float(m.group(2).strip()))
