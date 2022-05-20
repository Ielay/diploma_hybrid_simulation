package ru.spbu.jade2.behaviour;

import jade.core.behaviours.CyclicBehaviour;
import jade.lang.acl.ACLMessage;
import ru.spbu.jade2.entity.AgentInfo;
import ru.spbu.jade2.entity.AgentToAgentMsg;
import ru.spbu.jade2.agent.RobotSolutionAgent;
import ru.spbu.jade2.log.LoggerWrapper;

import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * @author lelay
 * @since 25.11.2020
 */
public class AzimuthWaiterBehaviour extends CyclicBehaviour {

    private static final LoggerWrapper log = new LoggerWrapper(AzimuthWaiterBehaviour.class);

    private final AtomicReference<AgentInfo> selfInfoRef;
    private final ConcurrentMap<String, AgentToAgentMsg> agentInfoById;

    public AzimuthWaiterBehaviour(RobotSolutionAgent agent) {
        super(agent);

        this.selfInfoRef = agent.getSelfInfoRef();
        this.agentInfoById = agent.getAgentInfoById();
    }

    @Override
    public void action() {
        while (this.getAgent().getCurQueueSize() > 0) {
            ACLMessage receivedMsg = this.getAgent().receive();
            if (isInformMessage(receivedMsg)) {
                String senderName = receivedMsg.getSender().getLocalName();

                log.debug(LoggerWrapper.Action.RECEIVE, receivedMsg);

                try {
                    AgentToAgentMsg receivedAgentToAgentMsg = (AgentToAgentMsg) receivedMsg.getContentObject();

                    agentInfoById.compute(senderName, (key, oldValue) -> {
                        if (oldValue == null) {
                            // add new value if it doesn't exist yet
                            return receivedAgentToAgentMsg;
                        } else {
                            // update old value if it already exists
                            oldValue.azimuth = receivedAgentToAgentMsg.azimuth;
                            oldValue.q = receivedAgentToAgentMsg.q;

                            return oldValue;
                        }
                    });
                } catch (Exception exc) {
                    log.error("Something bad happened", exc);
                    throw new RuntimeException(exc);
                }
            }
        }
    }

    private boolean isInformMessage(ACLMessage msg) {
        return msg != null && msg.getPerformative() == ACLMessage.INFORM;
    }
}
