from sensor_msgs.msg import CameraInfo
import rospy

def replace_camera_info_messages(bag, topic0, topic1, camera_info0, camera_info1):
    import os.path
    import rosbag
    orig = os.path.splitext(bag)[0] + '.orig.bag'
    os.rename(bag, orig)
    inbag = rosbag.Bag(orig, 'r')
    outbag = rosbag.Bag(bag, 'w')
    print("Starting to read!")
    i = 0
    for t, msg, ts in inbag.read_messages():
        if t == topic0:
            camera_info0.header.seq = msg.header.seq
            camera_info0.header.stamp = msg.header.stamp
            outbag.write(t, camera_info0, ts)
            print("Read cam0 topic #" + str(i))
            i = i + 1
        elif t == topic1:
            camera_info1.header.seq = msg.header.seq
            camera_info1.header.stamp = msg.header.stamp
            outbag.write(t, camera_info1, ts)
            print("Read cam1 topic #" + str(i))
            i = i + 1
        else:
            outbag.write(t, msg, ts)
    inbag.close()
    outbag.close()

def reformat_msg(msg, cam_idx):
    P = list(msg.P)
    D = list(msg.D)
    K = list(msg.K)
    if cam_idx == 0:
        D_params = [-0.03116674,  0.50057031, -7.69105705, 41.71286545]
        P_params = [1888.44515582, 1888.40009491,  613.18976514,  482.11894092]
    elif cam_idx == 1:
        D_params = [-0.10081622,   2.43900653, -26.79128779, 101.51121326]
        P_params = [1868.57412762, 1869.70165955,  573.22472505,  460.01067096]
        P[3] = -0.33
    D = D_params
    P[0] = P_params[0]
    P[5] = P_params[1]
    P[2] = P_params[2]
    P[6] = P_params[3]
    K[0] = P_params[0]
    K[4] = P_params[1]
    K[2] = P_params[2]
    K[5] = P_params[3]
    msg.D = tuple(D)
    msg.P = tuple(P)
    msg.K = tuple(K)
    msg.distortion_model = "equidistant"
    msg.height = 1024
    msg.width = 1224
    return msg


def main():
    rospy.init_node("camera_info_replacer")
    topic0 = "/camera_array/cam0/camera_info"
    topic1 = "/camera_array/cam1/camera_info"
    print("Waiting for topic!")
    msg0 = rospy.wait_for_message(topic0, CameraInfo)
    msg1 = rospy.wait_for_message(topic1, CameraInfo)

    msg0 = reformat_msg(msg0, 0)
    msg1 = reformat_msg(msg1, 1)

    print("Calling replace function!")
    replace_camera_info_messages("/media/bmchale/BMcHale_SSD/home/git/neu/bags/final_100.bag", topic0, topic1, msg0, msg1)

if __name__=="__main__":
    main()