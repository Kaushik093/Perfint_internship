import rospy
from std_msgs.msg import String

def talker():
    pub=rospy.Publisher('chatter',String,queue_size=10)
    # queue_size is a buffer
    # topic name-chatter , String is type of topic name

    # initialise ros node
    rospy.init_node('talker',anonymous=True)

    # talker is the name of the node and anonymous ensures every node is unique
    # if multiple talker nodes are created it adds and id to each talker node

    # setting loop rate
    rate=rospy.Rate(1)
    i=0
    while not rospy.is_shutdown():
        str="hello %s" % i
        rospy.loginfo(str)
        pub.publish(str)   
        rate.sleep()
        i=i+1
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
