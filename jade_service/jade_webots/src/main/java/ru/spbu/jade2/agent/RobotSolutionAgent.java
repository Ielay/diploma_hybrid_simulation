package ru.spbu.jade2.agent;

import jade.core.Agent;
import ros.RosBridge;
import ru.spbu.jade2.behaviour.AzimuthWaiterBehaviour;
import ru.spbu.jade2.behaviour.CalculateNewAzimuthBehaviour2;
import ru.spbu.jade2.entity.AgentInfo;
import ru.spbu.jade2.entity.AgentToAgentMsg;
import ru.spbu.jade2.log.LoggerWrapper;

import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * @author lelay
 * @since 23.11.2020
 */
public class RobotSolutionAgent extends Agent {

    private static final LoggerWrapper log = new LoggerWrapper(RobotSolutionAgent.class);

    private final AtomicReference<AgentInfo> selfInfoRef = new AtomicReference<>();

    private final ConcurrentMap<String, AgentToAgentMsg> agentInfoById = new ConcurrentHashMap<>();

    private RosBridge rosBridge;

    private List<String> linkedAgents;

    public List<String> getLinkedAgents() {
        return linkedAgents;
    }

    public AtomicReference<AgentInfo> getSelfInfoRef() {
        return selfInfoRef;
    }

    public ConcurrentMap<String, AgentToAgentMsg> getAgentInfoById() {
        return agentInfoById;
    }

    public RosBridge getRosBridge() {
        return rosBridge;
    }

    @Override
    protected void setup() {
        String agentName = this.getLocalName();

        Object[] args = this.getArguments();
        Map<String, List<String>> agentToLinkedAgents = (Map<String, List<String>>) args[0];
        long azimuthCalculationTimeStep = (long) args[1];
        this.linkedAgents = agentToLinkedAgents.get(agentName);
        this.rosBridge = (RosBridge) args[2];

        log.debug("Configuring {} agent", agentName);

        addBehaviour(
                new AzimuthWaiterBehaviour(
                        this
                )
        );
        addBehaviour(
                new CalculateNewAzimuthBehaviour2(
                        this,
                        azimuthCalculationTimeStep
                )
        );
    }
}
