import rospy
from sonia_msgs.srv import GetProcTreeList


def get_proc_tree_list():
    rospy.wait_for_service('/proc_mapping/get_proc_tree_list')
    try:
        service_handler = rospy.ServiceProxy('/proc_mapping/get_proc_tree_list', GetProcTreeList)
        result = service_handler()
        return result
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)


def get_proc_unit_list(proc_tree):
    pass


def get_proc_unit_parameter_list(proc_tree, proc_unit):
    pass


def get_proc_unit_parameter_value(proc_tree, proc_unit, parameter):
    pass


def set_proc_unit_parameter_value(proc_tree, proc_unit, parameter, value):
    pass


def change_current_proc_tree(proc_tree):
    pass
