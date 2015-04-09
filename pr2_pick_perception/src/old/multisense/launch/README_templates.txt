1. launch world (e.g. roslaunch Amazon_worlds multisense_shelf.launch )
2. rosrun laser_assembler laser_scan_assembler _fixed_frame:=/map _max_scans:=2400 scan:=/multisense_sl/laser/scan

3. go to devel/lib/trooper_multisense
4. ./laser_snapshotter_save _freq:=2 
5. ./save_cloud_topic cloud:=assembled_cloud _extract:=true _output:=view_0.pcd
