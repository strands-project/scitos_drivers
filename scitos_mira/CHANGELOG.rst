^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_mira
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2015-01-13)
------------------
* indigo-0.1.1
* Manual indigo version bump.
* Contributors: Chris Burbridge

Forthcoming
-----------
* added qt4 dep
* changelogs
* changed maintainer from Chris to Marc
* modifications required for new MIRA and Kinetic, plus setting MIRA_PATH from setup.bash
* MotorReset enabled
  fixes https://github.com/strands-project/scitos_robot/issues/77
* Contributors: Marc Hanheide

* changed maintainer from Chris to Marc
* modifications required for new MIRA and Kinetic, plus setting MIRA_PATH from setup.bash
* MotorReset enabled
  fixes https://github.com/strands-project/scitos_robot/issues/77
* Contributors: Marc Hanheide

0.1.9 (2016-01-28)
------------------
* changed ros::Time::now() to actual mira timestamp for the messaged taken from MIRA and published into ROS
* Contributors: g_gemignani

0.1.8 (2015-06-09)
------------------
* Add service to get persistent charger error log.
* fixing logic
* fix typo
* preventing the robot from moving when emergency_stop is active if new cmd_vel arrives
* Contributors: Chris Burbridge, Jaime Pulido Fentanes

0.1.7 (2015-05-19)
------------------
* Don't reset odometry on start.
  Allows restarting the mira interface node without messing up amcl.
* Contributors: Chris Burbridge

0.1.6 (2015-05-13)
------------------

0.1.5 (2015-04-22)
------------------
* Make magnetic barrier enable a normal parameter.
* Contributors: Chris Burbridge

0.1.4 (2015-04-13)
------------------
* Fixed indentatioN
* Added rear laser to the ebc cfgs
* Contributors: Nils Bore

0.1.3 (2015-03-24)
------------------
* indigo-0.1.2
* Update changelogs.
* indigo-0.1.1
* Manual indigo version bump.
* Contributors: Chris Burbridge

0.0.13 (2014-12-02)
-------------------

0.0.12 (2014-11-20)
-------------------
* Fix maintainer name.
* Make magnetic barrier a more secure stop.
* Contributors: Chris Burbridge

0.0.11 (2014-11-18)
-------------------

0.0.10 (2014-11-18)
-------------------

0.0.9 (2014-11-18)
------------------

0.0.8 (2014-11-14)
------------------
* Magnetic stripe detection added.
* MCU power controled as other EBC's, motor force reconfigurable parameter.
* Adding respawn to scitos node, removing launch files from scitos drivers, making scitos drivers a metapackage again
* Contributors: Jaime Pulido Fentanes, Tom Krajnik

0.0.6 (2014-11-11)
------------------
* changing OdometryInterval to 50 to avoid overshooting
* Contributors: Bruno Lacerda

0.0.5 (2014-11-09)
------------------
* final and tested version of loader
* new machine tag format
* scitos_mira.launch with new machine tags
* Contributors: Jaime Pulido Fentanes, strands

0.0.4 (2014-11-06)
------------------
* enabling all EBC ports per default
  fixes `#70 <https://github.com/strands-project/scitos_drivers/issues/70>`_
* Contributors: Marc Hanheide

0.0.3 (2014-11-06)
------------------
* added rpath
* Contributors: Marc Hanheide

0.0.2 (2014-10-14)
------------------
* added mira-scitos as dependency
* added default MIRA_PATH to use with debian package
* new SCITOS version
* Adding machine tags to mira, sick and ptu launch files
* made the SCITOSDriver config file an argument to use udev rules at UOL. Shouldn't effect anyone else
* Spinning in own thread seperate to publishing thread.
* Adding exception catching for mira parameter access. Issue `#23 <https://github.com/strands-project/scitos_drivers/issues/23>`_
* Adding exception catching for mira parameter access. Issue `#23 <https://github.com/strands-project/scitos_drivers/issues/23>`_
* Closing issue `#41 <https://github.com/strands-project/scitos_drivers/issues/41>`_. Changed the odometry interval to 20ms. This means that the odometry is sent every 20ms. This is the fastest rate I could achieve. Any faster and the odometry was not published any more. The resulting rate is ~47hz. We tested this odometry rate for quite some time and it does not seem to have any negative effects.
* Update README.md
* Adding conversion of MIRA debug output to ROS debug messages.
* Adding msgs dependency
* Adding bumper to motorstatus topic
* adding abilty to control eyelids in sync
* Update CMakeLists.txt
  including scitos_msgs generation before scitos_mira
* Adding motor status information publication
* Fixing boost::bind usage for MIRA callbacks
* Head lights controllable
* adding headlight callback
* Chaning head state publication frequency to 5hz to save CPU
* Tidy up
* Making SCITOS modules selectable from launch file.
* add launch file
* rename..
* rename scitos_driver=>scitos_mira
* rename scitos_driver to scitos_mira
* rename metapackage to scitos_drivers
* Made into catkin metapackage
* Contributors: Chris Burbridge, Christian Dondrup, Jaime Pulido Fentanes, Marc Hanheide, cburbridge
