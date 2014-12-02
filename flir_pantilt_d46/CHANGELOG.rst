^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flir_pantilt_d46
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.13 (2014-12-02)
-------------------

0.0.12 (2014-11-20)
-------------------

0.0.11 (2014-11-18)
-------------------

0.0.10 (2014-11-18)
-------------------

0.0.9 (2014-11-18)
------------------

0.0.8 (2014-11-14)
------------------

0.0.6 (2014-11-11)
------------------

0.0.5 (2014-11-09)
------------------
* final and tested version of loader
* new machine tag format
* added new machine tags to ptu launch file
* Contributors: Jaime Pulido Fentanes

0.0.4 (2014-11-06)
------------------

0.0.3 (2014-11-06)
------------------
* attempt to fix `#67 <https://github.com/strands-project/scitos_drivers/issues/67>`_ by adding install target
* Contributors: Marc Hanheide

0.0.2 (2014-10-14)
------------------
* Added preemption to ptu_action_server
* Adding machine tags to mira, sick and ptu launch files
* removed stupid log
* compelted bug fix after some git troubles
* pulled in
* fixed bug
* Revert "Merge pull request `#54 <https://github.com/strands-project/scitos_drivers/issues/54>`_ from marc-hanheide/hydro-devel"
  This reverts commit 71e00ddcabb0ee9981d59033a2cb7b505db08ab9, reversing
  changes made to 51742257dd76556f461efb567828a7d5108bec48. The changes result in
  the /ptu/state topic being broken. Probably because of line 325 in
  flir_pantilt_d46/src/ptu46_driver.cc.
* tested disabling of limits and added to launch file as default
* first shot on no limits
* PTU action server using parameterised joint names
* making PTU unit joint names parameterised
* updating ptu action server to use joint names
* make use of joint name to control joints independently. solves `#40 <https://github.com/strands-project/scitos_drivers/issues/40>`_
* Fixing action server startup
* rename launch
* move launch file to own dir
* prepare to move into scitos_drivers (scitos_mira)
* Contributors: Bob, Chris Burbridge, Jaime Pulido Fentanes, Marc Hanheide, Rares Ambrus, strands
