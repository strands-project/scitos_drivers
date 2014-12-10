#!/usr/bin/env python

import rospy
import sysmon
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class PCMonitor(object):
    def __init__(self, diagnostic_freq=0.2, pc_name="PC0", wlan_iface="wlan0"):
        self._diagnostic_freq = diagnostic_freq
        self._pc_name = pc_name
        self._wlan_mon = sysmon.WirelessMonitor(iface=wlan_iface)
        self._cpu_temp_mon = sysmon.CPUTemperature()
        self._cpu_usage_mon = sysmon.CPUUsage()
        self._bandwidth_mon = sysmon.NetworkBandwidthMonitor(wlan_iface)

        self._cpu_temp_mon.add_warn_condition("cpu0_temp",
                                         lambda x: True if x>80 else False,
                                         "CPU temperature too high!")
        self._cpu_temp_mon.add_warn_condition("cpu1_temp",
                                         lambda x: True if x>80 else False,
                                         "CPU temperature too high!")

        self._wlan_mon.add_warn_condition("signal_strength",
                                         lambda x: True if x<30 else False,
                                         "Wireless signal bad!")

        self._monitors = [self._cpu_temp_mon, self._cpu_usage_mon, self._bandwidth_mon, self._wlan_mon ]

        self._diagnostic_pub = rospy.Publisher("/diagnostics",DiagnosticArray)
        

    def spin(self):
        r = rospy.Rate(self._diagnostic_freq)
        while not rospy.is_shutdown():
            diag = DiagnosticArray()
            diag.header.stamp = rospy.get_rostime()
            for mon in self._monitors:
                d = DiagnosticStatus()
                d.name=mon.get_name()
                d.hardware_id = self._pc_name
                d.message=mon.get_field_value("status")
                d.level=mon.get_field_value("status_level")
                    

                for key in mon.get_fields():
                    p = KeyValue()
                    p.key = key
                    p.value = str(mon.get_field_value(key))
                    d.values.append(p)
                diag.status.append(d)
            self._diagnostic_pub.publish(diag)
            
            r.sleep()
            
        self._cpu_temp_mon.stop()
        self._cpu_usage_mon.stop()
        self._wlan_mon.stop()
        self._bandwidth_mon.stop()

if __name__ == '__main__':
    rospy.init_node('pc_monitor')
    wlan_iface = rospy.get_param("wireless_iface", "wlan0")
    pc_name = rospy.get_param("pc_name", "PC0")
    monitor = PCMonitor(pc_name=pc_name, wlan_iface=wlan_iface)
    monitor.spin()
