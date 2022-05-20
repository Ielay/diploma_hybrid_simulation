package ru.spbu.jade2.io.ros;

import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;

public class RosReceiver {

    private final RosBridge bridge;
    private final String topicName;

    public RosReceiver(RosBridge bridge, String robotId) {
        this.bridge = bridge;
        this.topicName = "/webots_to_jade/" + robotId + "/robot_info";
    }

    public void subscribe(RosListenDelegate logic) {
        SubscriptionRequestMsg subscriptionInfo = SubscriptionRequestMsg.generate(topicName)
                .setType("std_msgs/String")
                .setThrottleRate(1)
                .setQueueLength(1);

        bridge.subscribe(subscriptionInfo, logic);
    }

    public void unsubscribe() {
        bridge.unsubscribe(topicName);
    }
}
