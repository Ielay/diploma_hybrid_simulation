package ru.spbu.jade2.io.ros;

import ros.Publisher;
import ros.RosBridge;
import ros.msgs.std_msgs.PrimitiveMsg;

public class RosPublisher {

    private final RosBridge bridge;
    private final String topicName;
    private final Publisher pub;

    public RosPublisher(RosBridge bridge, String robotId) {
        this.bridge = bridge;
        this.topicName = "/jade_to_webots/" + robotId + "/new_azimuth";
        this.pub = new Publisher(topicName, "std_msgs/String", bridge);
    }

    public void publish(String msg) {
        pub.publish(new PrimitiveMsg<>(msg));
    }
}
