from status_monitor import  StatusMonitor
import re
import subprocess

class NetworkBandwidthMonitor(StatusMonitor):
    def __init__(self, iface):
        self._iface=iface
        self._freq=0.2
        super(NetworkBandwidthMonitor, self).__init__(self._freq)
        self._last_receive_total = self._last_transmit_total = 0

        self._name=iface + " Bandwidth"

    def get_fields(self):
        return ["receive_rate", "transmit_rate","receive_total","transmit_total"]
        
    def update(self):
        with open("/proc/net/dev") as f:
            stats = f.read()
        m =re.search("%s:\s+(\d+)\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+(\d+)\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+"%self._iface,stats)
        if m is None:
            self._status = "Network interface %s not known" % self._iface
            self._status_level = 1
        else:
            self._transmit_total = int(m.group(2))
            self._receive_total = int(m.group(1))
            self._receive_rate = (self._receive_total - self._last_receive_total)  * self._freq
            self._transmit_rate = (self._transmit_total - self._last_transmit_total) *  self._freq
            self._last_receive_total = self._receive_total
            self._last_transmit_total = self._transmit_total
        
        

class WirelessMonitor(StatusMonitor):
    def __init__(self, iface="wlan0"):
        self._iface=iface
        self._freq=0.2
        super(WirelessMonitor, self).__init__(self._freq)

        self._name=iface + " Connection"

    def get_fields(self):
        return ["connected", "network", "signal_strength", "bit_rate"]
        
    def update(self):
        iwconfig = subprocess.check_output(['iwconfig'],stderr=subprocess.STDOUT).decode("utf-8")
        m = re.search("%s.*ESSID:\"(\w+)\".*Bit Rate=(\d+).*Link Quality=(\d+)/(\d+)"%self._iface,iwconfig, re.DOTALL)
        if m is None:
            self._connected=False
            self._status = "Wireless interface %s not known" % self._iface
            self._status_level = 1
        else:
            self._connected=True
            self._network = m.group(1)
            self._signal_strength = int(m.group(3))/float(m.group(4))*100
            self._bit_rate=int(m.group(2))

        
        
    
