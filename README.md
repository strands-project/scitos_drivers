scitos_pc_monitor
=================

This package provides a node that monitors the health of the embedded scitos pc. Status updates are published as diagnostics messages on /diagnostics. These messages are collated by a diagnostics aggregator, which then allows them to be viewed on the scitos_dashboard.


Dependencies
------------
The following decencies are required for this package:

* mpstat, from the sysstat package
* sensors, from the lm-sensors

These are met by running
```
sudo apt-get install lm-sensors sysstat
```

Running
-------
The monitor is now started automatically when you use scitos_bringup. To start independently (assuming a roscore is existing somewhere, ie from scitos bringup):

```
rosrun scitos_pc_monitor pc_monitor.py
```

This then sends publishes on /diagnostics. To make use of the information, launch the diagnostics aggregator:

```
roslaunch scitos_bringup diagnostic_agg.launch
```

and view the message on the dashboard on your of-board pc:

```
rosrun scitos_dashboard scitos_dashboard.py
```

See https://github.com/strands-project/scitos_apps/tree/master/scitos_dashboard for dashboard documentation.

