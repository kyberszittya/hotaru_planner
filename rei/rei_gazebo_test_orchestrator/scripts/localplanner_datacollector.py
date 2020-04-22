import rospy

from datetime import datetime

from autoware_msgs.msg import ControlCommandStamped
from std_msgs.msg import Float64
from std_msgs.msg import Int32

from enum import Enum

class CollectorState(Enum):
    INIT = 1
    START_POINT = 2
    END_POINT = 3


class LocalPlannerDataCollector(object):

    def __init__(self):        
        self.output_csv = None
        # Parameters to be collected
        self.wheel_angle = None
        self.linear_velocity = None
        self.abs_lateral_distance = None
        self.closest_waypoint = -1
        self.last_wp_update = False
        self.collectorstate = CollectorState(CollectorState.INIT)
        # ROS subscribers
        self.sub_wp_lateral_distance = rospy.Subscriber("/wp_abs_lateral_distance", Float64, self.cbWpAbsLateralDistance)
        self.sub_vehicle_command = rospy.Subscriber("/ctrl_cmd", ControlCommandStamped, self.cbVehicleCommand)
        self.sub_closest_aypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cbClosestWaypoint)        

    def cbVehicleCommand(self, data):
        self.wheel_angle = data.cmd.steering_angle
        self.linear_velocity = data.cmd.linear_velocity

    def cbWpAbsLateralDistance(self, data):
        self.abs_lateral_distance = data.data
    
    def cbClosestWaypoint(self, data):
        self.closest_waypoint = data.data        
        if (self.closest_waypoint < 5):    
            if (not self.last_wp_update):
                if (self.collectorstate is CollectorState.INIT or self.collectorstate is CollectorState.END_POINT):
                    rospy.loginfo("START COLLECTION")
                    self.collectorstate = CollectorState.START_POINT
                    self.output_csv = open("data/datacollection_{0}.csv".format(datetime.now().strftime("%Y_%m_%d_%H_%M_%S")), "w")
                    self.output_csv.write("timestamp;wheel_angle;linear_velocity;abs_lateral_distance\n")
                    self.write_csv_timer = rospy.Timer(rospy.Duration(0.05), self.cbTimerWriteCmd)
                    self.last_wp_update = True
                else:
                    rospy.loginfo("END POINT")
                    self.collectorstate = CollectorState.END_POINT
                    self.write_csv_timer.shutdown()
                    self.output_csv.close()
                    self.last_wp_update = True
        else:
            self.last_wp_update = False

    def cbTimerWriteCmd(self, event):
        if (not self.output_csv is None):
            self.output_csv.write("{0};{1};{2};{3}\n".format(event.current_real, self.wheel_angle, self.linear_velocity, self.abs_lateral_distance))
    


def main():
    rospy.init_node("local_planner_data_collector")
    planner_data_collector = LocalPlannerDataCollector()
    rospy.spin()

if __name__=="__main__":
    main()