package ru.spbu.jade.converter;

import org.json.JSONObject;
import ru.spbu.jade.agent.AgentCoordinates;
import ru.spbu.jade.agent.AgentInfo;

public class AgentInfoConverter {

    private AgentInfoConverter() {

    }

    public static AgentInfo toObject(String json) {
        JSONObject jsonAgentInfo = new JSONObject(json);

        AgentInfo agentInfo = new AgentInfo();
        agentInfo.id = jsonAgentInfo.getString("id");
        agentInfo.azimuth = jsonAgentInfo.getInt("azimuth");

        AgentCoordinates coordinates = new AgentCoordinates();
        JSONObject jsonCoords = jsonAgentInfo.getJSONObject("coords");
        coordinates.x = jsonCoords.getDouble("x");
        coordinates.y = jsonCoords.getDouble("y");
        coordinates.z = jsonCoords.getDouble("z");

        agentInfo.coordinates = coordinates;

        return agentInfo;
    }

    public static String toJson(AgentInfo agentInfo) {
        JSONObject jsonAgentInfo = new JSONObject();
        jsonAgentInfo.put("id", agentInfo.id);
        jsonAgentInfo.put("azimuth", agentInfo.azimuth);

        JSONObject jsonCoords = new JSONObject();
        AgentCoordinates coords = agentInfo.coordinates;
        jsonCoords.put("x", coords.x);
        jsonCoords.put("y", coords.y);
        jsonCoords.put("z", coords.z);

        jsonAgentInfo.put("coords", jsonCoords);

        return jsonAgentInfo.toString();
    }
}
