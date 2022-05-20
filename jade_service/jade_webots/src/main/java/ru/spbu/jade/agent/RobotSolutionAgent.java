package ru.spbu.jade.agent;

import jade.core.Agent;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import ru.spbu.jade.behaviour.AzimuthWaiterBehaviour;
import ru.spbu.jade.behaviour.CalcAvgAzimuthBehaviour;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;

/**
 * @author lelay
 * @since 23.11.2020
 */
public class RobotSolutionAgent extends Agent {

    private static final Logger log = LoggerFactory.getLogger(RobotSolutionAgent.class);

    private final AtomicReference<AgentInfo> selfInfoRef = new AtomicReference<>();

    private final Map<String, AgentInfo> agentInfoById = new HashMap<>();
    private final ReentrantLock agentInfoMapLock = new ReentrantLock(true);

//    public static final Path AGENT_MEM_PATH = Paths.get("/home/lelay/programming/webots_res");
    public static final Path AGENT_MEM_PATH = Paths.get("C:\\diploma\\webots_messaging");
    public static final String INPUT_FILE_POSTFIX = "_mem_w_to_j.txt";
    public static final String OUTPUT_FILE_POSTFIX = "_mem_j_to_w.txt";

    private List<String> linkedAgents;

    public List<String> getLinkedAgents() {
        return linkedAgents;
    }

    public AtomicReference<AgentInfo> getSelfInfoRef() {
        return selfInfoRef;
    }

    public Map<String, AgentInfo> getAgentInfoById() {
        return agentInfoById;
    }

    public ReentrantLock getAgentInfoMapLock() {
        return agentInfoMapLock;
    }

    @Override
    protected void setup() {
        Object[] args = this.getArguments();
        Map<String, List<String>> agentToLinkedAgents = (Map<String, List<String>>) args[0];
        this.linkedAgents = agentToLinkedAgents.get(this.getLocalName());

//        clearOutputMemFile();

        addBehaviour(
                new AzimuthWaiterBehaviour(
                        this
                )
        );
        addBehaviour(
                new CalcAvgAzimuthBehaviour(
                        this,
                        TimeUnit.SECONDS.toMillis(5)
                )
        );
    }

    private void clearOutputMemFile() {
        final File outFile = AGENT_MEM_PATH
                .resolve(getLocalName() + OUTPUT_FILE_POSTFIX)
                .toFile();
        try (FileWriter outFileWriter = new FileWriter(outFile, false)) {
            outFileWriter.write("null");
            outFileWriter.flush();
        } catch (IOException e) {
            log.error("Output mem file can't be cleared", e);
        }
    }
}
