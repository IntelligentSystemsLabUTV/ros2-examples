^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rqt_image_view
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2022-04-07)
------------------
* Install headers to include/${PROJECT_NAME} (`#63 <https://github.com/ros-visualization/rqt_image_view/issues/63>`_)
* Using "sensor data" QoS profile with BEST_EFFORT (`#67 <https://github.com/ros-visualization/rqt_image_view/issues/67>`_)
* fix: modify qt deprecation warning (`#68 <https://github.com/ros-visualization/rqt_image_view/issues/68>`_)
* [FIX] add missing QComboBox to ui (`#66 <https://github.com/ros-visualization/rqt_image_view/issues/66>`_)
* [Rolling] Remember color scheme (`#64 <https://github.com/ros-visualization/rqt_image_view/issues/64>`_)
* Remember color scheme
* Contributors: Daisuke Nishimatsu, Karl Schulz, Matthijs van der Burgh, Shane Loretz, wodtko

1.1.2 (2022-01-20)
------------------
* Use image_transport and cv_bridge targets (`#62 <https://github.com/ros-visualization/rqt_image_view/issues/62>`_)
* Contributors: Shane Loretz

1.1.1 (2021-03-19)
------------------
* Updated maintainers.
* Contributors: Mabel Zhang

1.1.0 (2020-08-07)
------------------
* fix segfault on topic change (`#40 <https://github.com/ros-visualization/rqt_image_view/issues/40>`_)
* fix ImportError in image_publisher script (`#39 <https://github.com/ros-visualization/rqt_image_view/issues/39>`_)
* update include directives for image_transport to avoid deprecation warning (`#34 <https://github.com/ros-visualization/rqt_image_view/issues/34>`_)

1.0.4 (2020-01-10)
-------------------
* fix missing include for QSet (`#31 <https://github.com/ros-visualization/rqt_image_view/issues/31>`_)

1.0.3 (2019-10-23)
------------------
* Avoid deprecated API (`#28 <https://github.com/ros-visualization/rqt_image_view/issues/28>`_)
* Contributors: Karsten Knese

1.0.0 (2018-12-11)
------------------
* fix various compiler warnings (`#20 <https://github.com/ros-visualization/rqt_image_view/issues/20>`_)
* Notifying developers that rqt_image_view is not available for windows (`#19 <https://github.com/ros-visualization/rqt_image_view/issues/19>`_)
* Ros2 port (`#18 <https://github.com/ros-visualization/rqt_image_view/issues/18>`_)
* Contributors: Dirk Thomas, Mike Lautman

0.4.13 (2018-05-02)
-------------------
* add build dependency on Qt5 dev package (`#15 <https://github.com/ros-visualization/rqt_image_view/issues/15>`_)

0.4.12 (2018-05-02)
-------------------
* save and restore the smooth image check box state (`#13 <https://github.com/ros-visualization/rqt_image_view/issues/13>`_)
* allow image rotation in 90° steps (`#10 <https://github.com/ros-visualization/rqt_image_view/issues/10>`_)
* use Python distutils to install the global rqt_image_view executable (`#12 <https://github.com/ros-visualization/rqt_image_view/issues/12>`_)

0.4.11 (2017-10-25)
-------------------
* use catkin_install_python with scripts (`#8 <https://github.com/ros-visualization/rqt_image_view/issues/8>`_)

0.4.10 (2017-10-13)
-------------------
* add option to render grid (`#7 <https://github.com/ros-visualization/rqt_image_view/issues/7>`_)

0.4.9 (2017-07-27)
------------------
* reduce the size of the image border on the left and right side (`#5 <https://github.com/ros-visualization/rqt_image_view/issues/5>`_)
* avoid shrinking the image when the aspect ratio changes (`#4 <https://github.com/ros-visualization/rqt_image_view/issues/4>`_)

0.4.8 (2017-04-24)
------------------

0.4.7 (2017-03-02)
------------------

0.4.6 (2017-02-27)
------------------

0.4.5 (2017-02-03)
------------------

0.4.4 (2017-01-24)
------------------
* add checkbox for optional smooth image scaling (`#385 <https://github.com/ros-visualization/rqt_common_plugins/issues/385>`_)

0.4.3 (2016-11-02)
------------------
* generate UI headers in devel space to avoid CMake warning (`#401 <https://github.com/ros-visualization/rqt_common_plugins/pull/401>`_)

0.4.2 (2016-09-19)
------------------
* select existing topic (`#391 <https://github.com/ros-visualization/rqt_common_plugins/pull/391>`_)

0.4.1 (2016-05-16)
------------------
* add the possibility to publish mouse events (`#368 <https://github.com/ros-visualization/rqt_common_plugins/issues/368>`_)

0.4.0 (2016-04-27)
------------------
* Support Qt 5 (in Kinetic and higher) as well as Qt 4 (in Jade and earlier) (`#359 <https://github.com/ros-visualization/rqt_common_plugins/pull/359>`_)

0.3.13 (2016-03-08)
-------------------
* use proper icon for images
* add optional topic argument to rqt_image_view
* fix width of save-as-image button
* Contributors: Dirk Thomas, Vincent Rabaud

0.3.12 (2015-07-24)
-------------------
* Added button to save current image to file
* Contributors: Dirk Thomas

0.3.11 (2015-04-30)
-------------------
* fix image shrinking problem (`#291 <https://github.com/ros-visualization/rqt_common_plugins/issues/291>`_)

0.3.10 (2014-10-01)
-------------------
* update plugin scripts to use full name to avoid future naming collisions

0.3.9 (2014-08-18)
------------------

0.3.8 (2014-07-15)
------------------

0.3.7 (2014-07-11)
------------------

0.3.6 (2014-06-02)
------------------

0.3.5 (2014-05-07)
------------------
* list image transport topics if parent image topic is not available (`#215 <https://github.com/ros-visualization/rqt_common_plugins/issues/215>`_)

0.3.4 (2014-01-28)
------------------

0.3.3 (2014-01-08)
------------------
* add groups for rqt plugins, renamed some plugins (`#167 <https://github.com/ros-visualization/rqt_common_plugins/issues/167>`_)
* properly handle aligned images
* wrap cv calls in try-catch-block (`#201 <https://github.com/ros-visualization/rqt_common_plugins/issues/201>`_)

0.3.2 (2013-10-14)
------------------

0.3.1 (2013-10-09)
------------------
* fix event handling for rqt_image_view enabling to run multiple instances simultaneously (`#66 <https://github.com/ros-visualization/rqt_common_plugins/issues/66>`_)
* add rqt_image_view to global bin (`#168 <https://github.com/ros-visualization/rqt_common_plugins/issues/168>`_)

0.3.0 (2013-08-28)
------------------

0.2.17 (2013-07-04)
-------------------

0.2.16 (2013-04-09 13:33)
-------------------------

0.2.15 (2013-04-09 00:02)
-------------------------

0.2.14 (2013-03-14)
-------------------

0.2.13 (2013-03-11 22:14)
-------------------------

0.2.12 (2013-03-11 13:56)
-------------------------

0.2.11 (2013-03-08)
-------------------

0.2.10 (2013-01-22)
-------------------
* Optimized by taking more advantage of cv_bridge

0.2.9 (2013-01-17)
------------------

0.2.8 (2013-01-11)
------------------

0.2.7 (2012-12-24)
------------------

0.2.6 (2012-12-23)
------------------

0.2.5 (2012-12-21 19:11)
------------------------

0.2.4 (2012-12-21 01:13)
------------------------

0.2.3 (2012-12-21 00:24)
------------------------

0.2.2 (2012-12-20 18:29)
------------------------

0.2.1 (2012-12-20 17:47)
------------------------

0.2.0 (2012-12-20 17:39)
------------------------
* first release of this package into groovy
