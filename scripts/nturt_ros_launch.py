import roslaunch
import rospkg

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# get the file path for nturt_deploy_to_rpi
package_path = rospack.get_path('nturt_deploy_to_rpi')

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, [package_path + '/launch/nturt_ros.launch'])
launch.start()

# into endless loop until being shutdown
try:
    launch.spin()
finally:
    launch.shutdown()
