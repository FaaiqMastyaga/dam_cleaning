import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import actionlib
import random

class CoveragePlanner:
    def __init__(self) -> None:
        rospy.init_node('coverage_planner', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Subscribe to the map topic
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.feedback_subscriber = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.feedback_callback)
        self.map = None
        self.coverage_points = []
        self.current_point = 0
        self.current_goal = None
        self.max_retries = 3
        self.retry_count = 0

        rospy.spin()

    def map_callback(self, map_msg):
        if self.map is None:
            self.map = map_msg
            self.coverage_points = self.generate_coverage_points()
            self.move_to_next_point()

    def feedback_callback(self, feedback_msg):
        # Handle feedback here if needed
        pass

    def generate_coverage_points(self):
        points = []
        map_info = self.map.info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        width = map_info.width
        height = map_info.height

        scale = 3.555555
        offset = 0.1125
        step_size_small = 0.5  # Small step size for first 2 and last 2 rows
        buffer_distance = 0.5  # Buffer distance to avoid hitting walls

        # Determine the number of steps in x direction for small steps
        steps_x_small = int(width * resolution / step_size_small)
        steps_y = int(height * resolution / step_size_small)  # Total number of rows

        for i in range(steps_y):
            if i < 2 or i >= steps_y - 2:
                # For the first 2 and last 2 rows, use small steps
                for j in range(steps_x_small):
                    x = j * step_size_small
                    y = i * step_size_small  # Keep step size small for y as well
                    
                    if i % 2 == 1:  # Odd rows (right to left)
                        x = (steps_x_small - j - 1) * step_size_small

                    # Convert to world coordinates
                    world_x = (origin_x + x) / scale + offset
                    world_y = (origin_y + y) / scale + offset
                    
                    point = PoseStamped()
                    point.header.frame_id = 'map'
                    point.header.stamp = rospy.Time.now()
                    point.pose.position.x = world_x
                    point.pose.position.y = world_y

                    # Set orientation based on row direction
                    if i % 2 == 0:  # Left to right
                        point.pose.orientation.z = 0.0
                        point.pose.orientation.w = 1.0
                    else:  # Right to left
                        point.pose.orientation.z = 1.0
                        point.pose.orientation.w = 0.0

                    points.append(point)
            else:
                # For middle rows, divide the movement into three parts: start, x=-1.0, x=1.0, and end
                # Adjust buffer distance to keep points away from walls
                start_x = buffer_distance
                x_mid1 = (steps_x_small * resolution / 2) - 1.0  # Approx. x=-1.0
                x_mid2 = (steps_x_small * resolution / 2) + 1.0  # Approx. x=1.0
                end_x = (steps_x_small - 1) * step_size_small - buffer_distance  # End point in x

                # Convert start, mid, and end points to world coordinates
                start_world_x = (origin_x + start_x) / scale + offset
                mid1_world_x = (origin_x + x_mid1) / scale + offset
                mid2_world_x = (origin_x + x_mid2) / scale + offset
                end_world_x = (origin_x + end_x) / scale + offset
                world_y = (origin_y + i * step_size_small) / scale + offset

                if start_world_x < -2.65:
                    start_world_x = -2.65
                if end_world_x > 2.3:
                    end_world_x = 2.3
                    
                # Create start point
                start_point = PoseStamped()
                start_point.header.frame_id = 'map'
                start_point.header.stamp = rospy.Time.now()
                start_point.pose.position.x = start_world_x
                start_point.pose.position.y = world_y
                start_point.pose.orientation.z = 0.0
                start_point.pose.orientation.w = 1.0

                # Create mid points
                mid1_point = PoseStamped()
                mid1_point.header.frame_id = 'map'
                mid1_point.header.stamp = rospy.Time.now()
                mid1_point.pose.position.x = mid1_world_x
                mid1_point.pose.position.y = world_y
                mid1_point.pose.orientation.z = 0.0
                mid1_point.pose.orientation.w = 1.0

                mid2_point = PoseStamped()
                mid2_point.header.frame_id = 'map'
                mid2_point.header.stamp = rospy.Time.now()
                mid2_point.pose.position.x = mid2_world_x
                mid2_point.pose.position.y = world_y
                mid2_point.pose.orientation.z = 0.0
                mid2_point.pose.orientation.w = 1.0

                # Create end point
                end_point = PoseStamped()
                end_point.header.frame_id = 'map'
                end_point.header.stamp = rospy.Time.now()
                end_point.pose.position.x = end_world_x
                end_point.pose.position.y = world_y

                # Add the start, mid1, mid2, and end points to the list in the correct order
                if i % 2 == 0:  # Even rows: left to right
                    start_point.pose.orientation.z = 0.0
                    start_point.pose.orientation.w = 1.0
                    end_point.pose.orientation.z = 0.0
                    end_point.pose.orientation.w = 1.0
                    points.append(start_point)
                    points.append(mid1_point)
                    points.append(mid2_point)
                    points.append(end_point)
                else:  # Odd rows: right to left
                    end_point.pose.orientation.z = 1.0
                    end_point.pose.orientation.w = 0.0
                    points.append(end_point)
                    points.append(mid2_point)
                    points.append(mid1_point)
                    points.append(start_point)

        return points


    def move_to_next_point(self):
        if self.current_point < len(self.coverage_points):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.coverage_points[self.current_point].pose

            rospy.loginfo(f"Moving to point {self.current_point}")
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached point {self.current_point}")
                self.current_point += 1
                self.retry_count = 0  # Reset retry count
                self.move_to_next_point()
            else:
                rospy.logwarn(f"Failed to reach point {self.current_point}")
                self.retry_count += 1
                if self.retry_count < self.max_retries:
                    rospy.loginfo(f"Retrying point {self.current_point}")
                    self.move_to_next_point()
                else:
                    rospy.logwarn("Max retries reached. Choosing new goal.")
                    self.retry_count = 0
                    self.choose_new_goal()

        else:
            rospy.loginfo("Coverage completed")
            rospy.signal_shutdown("Coverage completed")

    def choose_new_goal(self):
        # Randomly choose a new goal from the coverage points
        if self.coverage_points:
            self.current_point = random.randint(0, len(self.coverage_points) - 1)
            self.move_to_next_point()
        else:
            rospy.logwarn("No coverage points available")
            rospy.signal_shutdown("No coverage points available")

if __name__ == '__main__':
    try:
        CoveragePlanner()
    except rospy.ROSInterruptException:
        pass
