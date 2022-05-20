package ru.spbu.jade.behaviour;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import ru.spbu.jade.agent.AgentInfo;
import ru.spbu.jade.agent.RobotSolutionAgent;
import ru.spbu.jade.converter.AgentInfoConverter;
import ru.spbu.jade.log.LoggerWrapper;

import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;

/**
 * @author lelay
 * @since 25.11.2020
 */
public class AzimuthWaiterBehaviour extends CyclicBehaviour {

    private static final LoggerWrapper log = new LoggerWrapper(AzimuthWaiterBehaviour.class);

    private final Map<String, AgentInfo> agentInfoById;
    private final ReentrantLock mapLock;

    public AzimuthWaiterBehaviour(RobotSolutionAgent agent) {
        super(agent);

        this.agentInfoById = agent.getAgentInfoById();
        this.mapLock = agent.getAgentInfoMapLock();
    }

    @Override
    public void action() {
        ACLMessage receivedMsg = this.getAgent().receive();
        if (isInformMessage(receivedMsg)) {
            String senderName = receivedMsg.getSender().getLocalName();

            log.debug(LoggerWrapper.Action.RECEIVE, receivedMsg);

            //TODO: refactoring - use concurrent map and atomic operations
            mapLock.lock();
            try {
                String msg = receivedMsg.getContent();
                log.debug("Received msg: {}", msg);
                AgentInfo receivedAgentInfo = AgentInfoConverter.toObject(msg);

                if (!agentInfoById.containsKey(senderName)) {
                    agentInfoById.put(senderName, receivedAgentInfo);
                } else {
                    AgentInfo agentInfo = agentInfoById.get(senderName);
                    agentInfo.azimuth = receivedAgentInfo.azimuth;
                    agentInfo.coordinates = receivedAgentInfo.coordinates;
                }
            } catch (Exception exc) {
                log.error("Something bad happened", exc);
                throw exc;
            } finally {
                mapLock.unlock();
            }
        }
    }

    private boolean isInformMessage(ACLMessage msg) {
        return msg != null && msg.getPerformative() == ACLMessage.INFORM;
    }
}
