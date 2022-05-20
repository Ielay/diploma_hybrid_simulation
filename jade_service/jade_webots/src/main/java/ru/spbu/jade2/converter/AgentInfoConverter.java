package ru.spbu.jade2.converter;

import org.json.JSONObject;
import ru.spbu.jade2.entity.AgentInfo;
import ru.spbu.jade2.entity.AgentToAgentMsg;

public class AgentInfoConverter {

    private AgentInfoConverter() {

    }

    public static AgentInfo toObjectFromJSON(String json) {
        JSONObject jsonMsg = new JSONObject(json);

        AgentInfo msg = new AgentInfo();
        msg.id = String.valueOf(jsonMsg.getInt("id"));
        msg.azimuth = jsonMsg.getDouble("azimuth");
        msg.q = jsonMsg.getDouble("q");
        msg.neighbours = RobotInfoConverter.intArrayOf(jsonMsg.getString("neighbours"));
        msg.light = RobotInfoConverter.doubleArrayOf(jsonMsg.getString("light"));
        msg.d = jsonMsg.getDouble("d");

        return msg;
    }

    public static AgentToAgentMsg toAgentToAgentMsgObject(String json) {
        JSONObject jsonMsg = new JSONObject(json);

        AgentToAgentMsg msg = new AgentToAgentMsg();
        msg.id = jsonMsg.getString("id");
        msg.azimuth = jsonMsg.getDouble("azimuth");
        msg.q = jsonMsg.getDouble("q");

        return msg;
    }

    public static String agentToAgentMsgToJson(AgentToAgentMsg msg) {
        JSONObject jsonAgentInfo = new JSONObject();
        jsonAgentInfo.put("id", msg.id);
        jsonAgentInfo.put("azimuth", msg.azimuth);
        jsonAgentInfo.put("q", msg.q);

        return jsonAgentInfo.toString();
    }
}
