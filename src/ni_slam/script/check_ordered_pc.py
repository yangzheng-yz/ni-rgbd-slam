import rosbag
import sensor_msgs.point_cloud2 as pc2

def is_ordered_cloud(cloud):
    return cloud.width * cloud.height > 0 and cloud.is_dense == False

def main():
    bag_file = "/home/zheng/projects/ni_slam_ws/src/ni_slam/bag/icl_lrkt1.bag"
    pointcloud_topic = "/camera/rgb/pointcloud"

    ordered_clouds = 0
    unordered_clouds = 0

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[pointcloud_topic]):
            if is_ordered_cloud(msg):
                ordered_clouds += 1
                print("Ordered point cloud found at time:", t)
            else:
                unordered_clouds += 1

    print("Ordered clouds: {}, Unordered clouds: {}".format(ordered_clouds, unordered_clouds))

if __name__ == "__main__":
    main()
