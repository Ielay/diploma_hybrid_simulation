package ru.spbu.jade2.entity;

import java.io.Serializable;

public class AgentToAgentMsg implements Serializable {

    public String id;

    public Double azimuth;

    public Double q;

    public AgentToAgentMsg() {

    }

    public AgentToAgentMsg(String id, Double azimuth, Double q) {
        this.id = id;
        this.azimuth = azimuth;
        this.q = q;
    }
}
