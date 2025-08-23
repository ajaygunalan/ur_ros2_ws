import rosbag
import tf
import rospy
from tf import TransformListener
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped

def extract_transform(bag_path, source_frame, target_frame):
    # Initialize a ROS node
    rospy.init_node('tf_extractor', anonymous=True)

    # Create a TransformListener to handle the tf data
    listener = TransformListener()
    tf_pub = rospy.Publisher('/tf', tfMessage, queue_size=10)

    # Open the bag file
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/tf_static']):
            print('Reading latest transform at time {}'.format(t.to_sec()))
            print(msg)
            tf_pub.publish(msg)

        i0 = 0

        for topic, msg, t in bag.read_messages(topics=['/tf']):
            print('Reading latest transform at time {}'.format(t.to_sec()))
            tf_pub.publish(msg)

        try:
            # Query the transformation between source_frame and target_frame
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            print(f"Transform from {source_frame} to {target_frame} at time {t.to_sec()}:")
            print("Translation:", trans)
            print("Rotation (quaternion):", rot)

            # Optional: Convert quaternion to Euler angles
            euler_angles = euler_from_quaternion(rot)
            print("Rotation (Euler angles):", euler_angles)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Error fetching transform:", e)
            # break

        i0 = i0 + 1

        # if i0 >= 100:
            # break

if __name__ == '__main__':
    # extract_transform('/home/mardava/Projects/robots/ur5_e_ws/bag/nethra_bag/fixed/2024-09-25-14-35-34.bag', 'base_link', 'wrist_3_link')
    extract_transform('/data/mardava/Projects/robots/ur5_e_ws/bag/20241106_animal_study/fixed/sys_a_2024-11-06-11-59-57.bag', 'p42v_link1', 'base_link')
