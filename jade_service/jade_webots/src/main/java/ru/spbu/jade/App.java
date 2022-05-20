package ru.spbu.jade;

import jade.core.Agent;
import jade.wrapper.AgentContainer;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import ru.spbu.jade.agent.RobotSolutionAgent;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * @author lelay
 * @since 15.11.2020
 */
public class App {

    private static final Logger log = LoggerFactory.getLogger(App.class);

    private static final String AGENT_NAME_PREFIX = "agent_";

    private static List<String> allAgents = IntStream.rangeClosed(0, 9)
            .mapToObj(i -> AGENT_NAME_PREFIX + i)
            .collect(Collectors.toList());
    
    private static List<String> allExceptPassed(String agent) {
        List<String> agents = new ArrayList<>(allAgents);

        agents.remove(agent);

        return agents;
    }

    public static void main(String[] args) {
        Runtime runtime = Runtime.instance();

        Profile profile = new ProfileImpl();
        profile.setParameter(Profile.MAIN_HOST, "localhost");
        profile.setParameter(Profile.MAIN_PORT, "10098");
        profile.setParameter(Profile.GUI, "false");

        AgentContainer agentContainer = runtime.createMainContainer(profile);

        Map<String, List<String>> agentToLinkedAgents = allAgents.stream()
                .map(id -> Pair.of(id, allExceptPassed(id)))
                .collect(Collectors.toMap(pair -> pair.getKey(), pair -> pair.getValue()));

        Object[] agentArgs = new Object[1];
        agentArgs[0] = agentToLinkedAgents;

        Class<? extends Agent> agentClass = RobotSolutionAgent.class;

        try {
            for (String agentName : allAgents) {
                AgentController agent = agentContainer.createNewAgent(
                        agentName,
                        agentClass.getName(),
                        agentArgs
                );
                agent.start();
            }
        } catch (StaleProxyException e) {
            log.error("ERROR agent creation!", e);
        }
    }
}
