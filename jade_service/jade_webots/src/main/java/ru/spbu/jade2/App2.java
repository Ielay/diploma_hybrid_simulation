package ru.spbu.jade2;

import jade.core.Agent;
import jade.core.Profile;
import jade.core.ProfileImpl;
import jade.core.Runtime;
import jade.wrapper.AgentContainer;
import jade.wrapper.AgentController;
import jade.wrapper.StaleProxyException;
import org.apache.commons.lang3.tuple.Pair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import ros.RosBridge;
import ru.spbu.jade2.agent.RobotSolutionAgent;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * @author lelay
 * @since 15.11.2020
 */
public class App2 {

    private static final Logger log = LoggerFactory.getLogger(App2.class);
    private static final long AZIMUTH_CALCULATION_TIME_STEP = 100;

    private static List<String> allExceptPassed(List<String> allAgents, String agent) {
        List<String> agents = new ArrayList<>(allAgents);
        agents.remove(agent);
        return agents;
    }

    public static void main(String[] args) {
        if (args.length != 6) {
            throw new RuntimeException("Error: expected 6 arguments. Actual args: " + Arrays.toString(args));
        }

        Runtime runtime = Runtime.instance();

        final int firstRobotNumber = Integer.parseInt(args[0]);
        final int robotsNumber = Integer.parseInt(args[1]);
        final String rosBridgeURI = args[2];
        final boolean isMainContainer = Boolean.parseBoolean(args[3]);
        final String mainContainerHost = args[4];
        final String mainContainerPort = args[5];

        final List<String> allAgents = IntStream.range(firstRobotNumber, firstRobotNumber + robotsNumber)
                .mapToObj(String::valueOf)
                .collect(Collectors.toList());

        Profile profile = new ProfileImpl();
        profile.setParameter(Profile.MAIN_HOST, mainContainerHost);
        profile.setParameter(Profile.MAIN_PORT, mainContainerPort);
        profile.setParameter(Profile.GUI, "false");

        AgentContainer agentContainer;
        if (isMainContainer) {
            agentContainer = runtime.createMainContainer(profile);
        } else {
            agentContainer = runtime.createAgentContainer(profile);
        }

        Map<String, List<String>> agentToLinkedAgents = allAgents.stream()
                .map(id -> Pair.of(id, allExceptPassed(allAgents, id)))
                .collect(Collectors.toMap(Pair::getKey, Pair::getValue));

        RosBridge rosBridge = new RosBridge();
        rosBridge.connect(rosBridgeURI, true);

        Object[] agentArgs = new Object[3];
        agentArgs[0] = agentToLinkedAgents;
        agentArgs[1] = AZIMUTH_CALCULATION_TIME_STEP;
        agentArgs[2] = rosBridge;

        createAndStartAgentsInContainer(agentContainer, allAgents, agentArgs);
    }

    private static void createAndStartAgentsInContainer(AgentContainer agentContainer, List<String> allAgents, Object[] agentArgs) {
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
