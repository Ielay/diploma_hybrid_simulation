package ru.spbu.jade.behaviour;

import jade.core.AID;
import jade.core.behaviours.TickerBehaviour;
import jade.lang.acl.ACLMessage;
import org.apache.commons.lang3.tuple.Pair;
import ru.spbu.jade.agent.AgentCoordinates;
import ru.spbu.jade.agent.AgentInfo;
import ru.spbu.jade.agent.RobotSolutionAgent;
import ru.spbu.jade.converter.AgentInfoConverter;
import ru.spbu.jade.io.ReadModule;
import ru.spbu.jade.io.WriteModule;
import ru.spbu.jade.io.fs.FileSystemReadModule;
import ru.spbu.jade.io.fs.FileSystemWriteModule;
import ru.spbu.jade.log.LoggerWrapper;

import java.io.File;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

/**
 * 1. read itself azimuth from input file
 * 2. calc avg azimuth
 * 3. write calculated azimuth to output file
 *
 * @author lelay
 * @since 23.11.2020
 */
public class CalcAvgAzimuthBehaviour extends TickerBehaviour {

    private static final LoggerWrapper log = new LoggerWrapper(CalcAvgAzimuthBehaviour.class);

    private final AtomicReference<AgentInfo> selfInfoRef;

    private final Map<String, AgentInfo> agentInfoById;
    private final ReentrantLock agentInfoMapLock;

    private final List<String> linkedAgents;

    private final ReadModule inDataReader;
    private final WriteModule outDataWriter;

    private static final Double RADIUS_METERS = 5.0;
    private static final Integer MAX_NUMBER_OF_NEAREST_AGENTS = 5;

    public CalcAvgAzimuthBehaviour(RobotSolutionAgent agent, long period) {
        super(agent, period);

        this.selfInfoRef = agent.getSelfInfoRef();

        this.agentInfoById = agent.getAgentInfoById();
        this.agentInfoMapLock = agent.getAgentInfoMapLock();

        this.linkedAgents = agent.getLinkedAgents();

        String agentName = agent.getLocalName();
        File inDataFile = RobotSolutionAgent.AGENT_MEM_PATH
                .resolve(agentName + RobotSolutionAgent.INPUT_FILE_POSTFIX)
                .toFile();
        File outDataFile = RobotSolutionAgent.AGENT_MEM_PATH
                .resolve(agentName + RobotSolutionAgent.OUTPUT_FILE_POSTFIX)
                .toFile();
        this.inDataReader = new FileSystemReadModule(inDataFile);
        this.outDataWriter = new FileSystemWriteModule(outDataFile);
    }

    @Override
    protected void onTick() {
        try {
            //TODO: the sending of self azimuth to nearest agents should be moved to independent behaviour
            AgentInfo agentInfo = readAgentInfoFromMemoryFile();
            if (agentInfo == null) {
                log.debug("No azimuth is provided yet");
                return;
            }
            selfInfoRef.set(agentInfo);

            sendAgentInfoToAllAgents(agentInfo);

//            sendAgentInfoToNearestAgents(agentInfo, nearestAgents);
//            log.debug("Nearest agents: ", nearestAgents.stream().map(agent -> agent.id).collect(Collectors.joining(", ")));

            //TODO 2022: why did I wrote this?
//            Thread.sleep(2500);

            agentInfoMapLock.lock();
            try {
                List<AgentInfo> nearestAgents = getNearestAgents(agentInfo);
                log.debug("Nearest agents: {}", nearestAgents.stream().map(agent -> agent.id).collect(Collectors.joining(", ")));

                int avgAzimuth = calcAvgAzimuth(nearestAgents);
                log.debug("Avg azimuth on '{}' is '{}'", this.getAgent().getLocalName(), avgAzimuth);

                writeAvgAzimuthToMemoryFile(avgAzimuth);

                //TODO: should this state be cleared actually? shouldn't the agent remember what state its neighbours had?
                clearAzimuths();
            } finally {
                agentInfoMapLock.unlock();
            }
        } catch (Throwable exc) {
            log.error("Error while calculating azimuth", exc);
            //TODO: exception handling
        }
    }

    private AgentInfo readAgentInfoFromMemoryFile() {
        try {
            String agentInfoSerialized = inDataReader.readValue();
            log.debug("Read agentInfo: {}", agentInfoSerialized);
            if (agentInfoSerialized.equals("null")) {
                return null;
            }

            AgentInfo agentInfo = AgentInfoConverter.toObject(agentInfoSerialized);

            selfInfoRef.set(agentInfo);

            return agentInfo;
        } catch (Throwable e) {
            throw new RuntimeException("Error reading itself azimuth", e);
        }
    }

    private void sendAgentInfoToAllAgents(AgentInfo agentInfo) {
        ACLMessage sendingMsg = new ACLMessage(ACLMessage.INFORM);
        for (String agentId : linkedAgents) {
            sendingMsg.addReceiver(new AID(agentId, AID.ISLOCALNAME));
        }

        sendingMsg.setContent(AgentInfoConverter.toJson(agentInfo));
        sendingMsg.setSender(this.getAgent().getAID());

        log.debug(LoggerWrapper.Action.SEND, sendingMsg);

        this.getAgent().send(sendingMsg);
    }

    private void sendAgentInfoToNearestAgents(AgentInfo agentInfo, List<AgentInfo> nearestAgents) {
        ACLMessage sendingMsg = new ACLMessage(ACLMessage.INFORM);
        for (AgentInfo nearestAgent : nearestAgents) {
            sendingMsg.addReceiver(new AID(nearestAgent.id, AID.ISLOCALNAME));
        }

        sendingMsg.setContent(AgentInfoConverter.toJson(agentInfo));
        // jade would set sender itself, but we are doing it in logging purpose
        sendingMsg.setSender(this.getAgent().getAID());

        log.debug(LoggerWrapper.Action.SEND, sendingMsg);

        this.getAgent().send(sendingMsg);
    }

    private List<AgentInfo> getNearestAgents(AgentInfo selfInfo) {
        return agentInfoById.values().stream()
                .map(agentInfo -> Pair.of(agentInfo, getDistanceBetweenAgents(selfInfo, agentInfo)))
                .filter(pair -> isAgentWithinRadius(pair.getValue()))
                .sorted(Comparator.comparingDouble(Pair::getValue))
                //.limit(MAX_NUMBER_OF_NEAREST_AGENTS)
                .map(Pair::getKey)
                .collect(Collectors.toList());
    }

    private boolean isAgentWithinRadius(double distance) {
        return distance <= RADIUS_METERS;
    }

    private Double getDistanceBetweenAgents(AgentInfo infoOne, AgentInfo infoTwo) {
        AgentCoordinates agentOneCoords = infoOne.coordinates;
        AgentCoordinates agentTwoCoords = infoTwo.coordinates;

        double xDiff = agentOneCoords.x - agentTwoCoords.x;
        double yDiff = agentOneCoords.y - agentTwoCoords.y;
        double zDiff = agentOneCoords.z - agentTwoCoords.z;

        return Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2) + Math.pow(zDiff, 2));
    }

//    private int calcAvgAzimuthViaLocalVotingProtocol(List<AgentInfo> nearestAgents) {
//
//    }

    //TODO: there is a proper way to calculate avg azimuth: https://rosettacode.org/wiki/Averages/Mean_angle#Java
    private int calcAvgAzimuth(List<AgentInfo> nearestAgents) {
        Map<String, AgentInfo> nearestAgentById = nearestAgents.stream()
                .map(agentInfo -> Pair.of(agentInfo.id, agentInfo))
                .collect(Collectors.toMap(Pair::getKey, Pair::getValue));

        int avgAzimuth = selfInfoRef.get().azimuth;
        for (Map.Entry<String, AgentInfo> entry : nearestAgentById.entrySet()) {
            AgentInfo nearestAgentInfo = entry.getValue();
            avgAzimuth += nearestAgentInfo.azimuth;
        }

        avgAzimuth /= nearestAgentById.size() + 1;

        return avgAzimuth;
    }

    private void writeAvgAzimuthToMemoryFile(int avgAzimuth) {
        outDataWriter.writeValue(String.valueOf(avgAzimuth));
    }

    private void clearAzimuths() {
        agentInfoById.clear();
    }
}
