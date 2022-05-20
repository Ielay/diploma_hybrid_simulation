package ru.spbu.jade2.behaviour;

import jade.core.AID;
import jade.core.behaviours.TickerBehaviour;
import jade.lang.acl.ACLMessage;
import ros.RosBridge;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;
import ru.spbu.jade2.agent.RobotSolutionAgent;
import ru.spbu.jade2.converter.AgentInfoConverter;
import ru.spbu.jade2.entity.AgentInfo;
import ru.spbu.jade2.entity.AgentToAgentMsg;
import ru.spbu.jade2.io.ros.RosPublisher;
import ru.spbu.jade2.io.ros.RosReceiver;
import ru.spbu.jade2.log.LoggerWrapper;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

/**
 * 1. read itself azimuth
 * 2. calc azimuth
 * 3. write calculated azimuth
 *
 * @author lelay
 * @since 23.11.2020
 */
public class CalculateNewAzimuthBehaviour2 extends TickerBehaviour {

    private static final LoggerWrapper log = new LoggerWrapper(CalculateNewAzimuthBehaviour2.class);

    private final AtomicReference<AgentInfo> selfInfoRef;
    private final ConcurrentMap<String, AgentToAgentMsg> agentInfoById;

    private final List<String> linkedAgents;
    private final Integer numberOfRobotsInHiveExceptItself;

    private final String agentName;
    private final RosBridge rosBridge;
    private final RosPublisher publisher;
    private final RosReceiver receiver;

    private static final Integer MAX_NUMBER_OF_NEAREST_AGENTS = 5;

    private static final double Q_OF_UNREACHABLE_AGENT = -1;

    public CalculateNewAzimuthBehaviour2(RobotSolutionAgent agent, long period) {
        super(agent, period);

        this.selfInfoRef = agent.getSelfInfoRef();
        this.agentInfoById = agent.getAgentInfoById();

        this.linkedAgents = agent.getLinkedAgents();
        this.numberOfRobotsInHiveExceptItself = linkedAgents.size();

//        this.redisSharedPool = agent.getRedisSharedPool();

        this.agentName = agent.getLocalName();

        this.rosBridge = agent.getRosBridge();
        this.publisher = new RosPublisher(rosBridge, agentName);
        this.receiver = new RosReceiver(rosBridge, agentName);
        receiver.subscribe((data, stringRep) -> {
            MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
            PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);

            AgentInfo agentInfo = AgentInfoConverter.toObjectFromJSON(msg.data);
            selfInfoRef.set(agentInfo);
        });
    }

    @Override
    protected void onTick() {
        try {
            AgentInfo agentItselfInfo = selfInfoRef.get();
            if (agentItselfInfo == null) {
                log.debug("No azimuth is provided yet");
                return;
            }
            selfInfoRef.set(agentItselfInfo);

            List<Integer> nearestNeighbours = getNearestAgents(agentItselfInfo);

            sendAgentInfoToNearestAgents(agentItselfInfo, nearestNeighbours);

            Map<String, AgentToAgentMsg> nearestAgents = agentInfoById.entrySet().stream()
                    .filter(entry -> nearestNeighbours.contains(Integer.valueOf(entry.getValue().id)))
                    .collect(Collectors.toMap(entry -> entry.getKey(), entry -> entry.getValue()));
            log.info("Agent - {}. Nearest agents number: {}", agentItselfInfo.id, nearestAgents.size());

            //Расчитываем желаемый азимут
            double maxLight = findMax(agentItselfInfo.light);
            int dbearing = calc_dbearing(agentItselfInfo.azimuth, agentItselfInfo.light, maxLight);

            //Принимаем сообщение
            //TODO v_p: поправить имена переменных под java-style
            int k_t = numberOfRobotsInHiveExceptItself + 1;
            // bearing_n[i][0] - bearing
            // bearing_n[i][1] - q
            double[][] bearing_n = new double[numberOfRobotsInHiveExceptItself + 1][2];

            for (int i = 0; i < numberOfRobotsInHiveExceptItself + 1; ++i) {
                String strKey = String.valueOf(i);
                if (nearestAgents.containsKey(strKey)) {
                    AgentToAgentMsg msg = nearestAgents.get(strKey);

                    bearing_n[i][0] = msg.azimuth;
                    bearing_n[i][1] = msg.q;
                    log.debug("Bearing_n: {}; {}", bearing_n[i][0], bearing_n[i][1]);
                } else {
                    k_t -= 1;
                    bearing_n[i][1] = Q_OF_UNREACHABLE_AGENT;
                }
            }
            log.debug("k_t = {}", k_t);

            //Расчитываем sigma
            double alpha = 0.8;
            double deltaq = 0;
            for (int i = 0; i < numberOfRobotsInHiveExceptItself + 1; ++i) {
                if (bearing_n[i][1] != Q_OF_UNREACHABLE_AGENT) {
                    deltaq += bearing_n[i][1] - agentItselfInfo.q;
                }
            }
            if (k_t == 0) {
                k_t = 1;
            }

            log.debug("deltaq = {}", deltaq);
            double sigma_t = (alpha * deltaq) / k_t;

            // Расчитываем гамма^i_t
            if (agentItselfInfo.q == 0) {
                agentItselfInfo.q = 0.01;
            }
            double gamma_t = 1 / (agentItselfInfo.q + sigma_t);

            // Расчитываем сумму разностей
            double cos_delta_sum = 0;
            double sin_delta_sum = 0;
            final double azimuthToRadians = Math.toRadians(agentItselfInfo.azimuth);
            final double dbearingToRadians = Math.toRadians(dbearing);
            double cos_bearing = Math.cos(azimuthToRadians);
            double sin_bearing = Math.sin(azimuthToRadians);
            double cos_dbearing = Math.cos(dbearingToRadians);
            double sin_dbearing = Math.sin(dbearingToRadians);

            for (int i = 0; i < numberOfRobotsInHiveExceptItself + 1; ++i) {
                if (bearing_n[i][1] != -1) {
                    final double bearing_nToRadians = Math.toRadians(bearing_n[i][0]);
                    double cos_bearingn = Math.cos(bearing_nToRadians);
                    double sin_bearingn = Math.sin(bearing_nToRadians);
                    cos_delta_sum += cos_bearingn * bearing_n[i][1] - cos_bearing * agentItselfInfo.q;
                    sin_delta_sum += sin_bearingn * bearing_n[i][1] - sin_bearing * agentItselfInfo.q;
                }
            }

            //Расчитываем курс в группе dbearingG исходя из данных группы
            // alpha - коэфициент, p - уверенность к курсу при пересчете от группы
            double dbearingG = 0;
            if (k_t == 0) {
                dbearingG = dbearing;
            } else {
                double cos_db_G = cos_dbearing * (1 - (sigma_t * gamma_t)) + (alpha * gamma_t * cos_delta_sum) / k_t;
                double sin_db_G = sin_dbearing * (1 - (sigma_t * gamma_t)) + (alpha * gamma_t * sin_delta_sum) / k_t;

                if (cos_db_G < -1) {
                    cos_db_G = -1;
                }
                if (cos_db_G > 1) {
                    cos_db_G = 1;
                }
                if (cos_db_G > 0 && sin_db_G > 0) {
                    dbearingG = Math.toDegrees(Math.acos(cos_db_G));
                } else if (cos_db_G > 0 && sin_db_G < 0) {
                    dbearingG = 360 - Math.toDegrees(Math.acos(cos_db_G));
                } else if (cos_db_G < 0 && sin_db_G > 0) {
                    dbearingG = 180 - Math.toDegrees(Math.acos(cos_db_G));
                } else if (cos_db_G < 0 && sin_db_G < 0) {
                    dbearingG = 180 + Math.toDegrees(Math.acos(cos_db_G));
                }
            }
            if (dbearingG > 360) {
                dbearingG = dbearingG - 360;
            }

            sendNewAzimuthToWebots(dbearingG);

            //clearAzimuths();
        } catch (Throwable exc) {
            log.error("Error while calculating azimuth", exc);
        }
    }

    private boolean allEqual(Double[] values) {
        double firstValue = values[0];
        for (int i = 1; i < values.length; ++i) {
            if (firstValue != values[i]) {
                return false;
            }
        }
        return true;
    }

    private double findMax(Double[] values) {
        double max = values[0];
        for (double value : values) {
            max = value;
        }

        return max;
    }

    private int calc_dbearing(double bearing, Double[] light, double light_max) {
        double dbearing = bearing;

        if (light_max != 0) {
            double a = 0;
            double b = 0;
            if (light_max == light[0]) {
                a = light[0] - light[3];
                b = light[0] - light[1];

                if (a < b && light[3] != 0) {
                    dbearing = bearing - 45 + (light[0] * 90) / (light[0] + light[3]);
                } else if (b < a && light[1] != 0) {
                    dbearing = bearing + 45 + (light[1] * 90) / (light[0] + light[1]);
                } else {
                    dbearing = bearing + 45;
                }
            } else if (light_max == light[1]) {
                a = light[1] - light[0];
                b = light[1] - light[2];

                if (a <= b && light[0] != 0) {
                    dbearing = bearing + 45 + (light[1] * 90) / (light[0] + light[1]);
                } else if (b < a && light[2] != 0){
                    dbearing = bearing + 135 + (light[2] * 90) / (light[2] + light[1]);
                } else{
                    dbearing = bearing + 135;
                }
            } else if (light_max == light[2]) {
                a = light[2] - light[1];
                b = light[2] - light[3];

                if (a <= b && light[1] != 0){
                    dbearing = bearing + 135 + (light[2] * 90) / (light[2] + light[1]);
                } else if (b < a && light[3] != 0){
                    dbearing = bearing + 225 + (light[3] * 90) / (light[2] + light[3]);
                } else {
                    dbearing = bearing + 225;
                }
            } else if (light_max == light[3]) {
                a = light[3] - light[2];
                b = light[3] - light[0];

                if (a <= b && light[2] != 0){
                    dbearing = bearing + 225 + (light[3] * 90) / (light[2] + light[3]);
                } else if (b < a && light[0] != 0 ){
                    dbearing = bearing + 315 + (light[0] * 90) / (light[0] + light[3]);
                } else {
                    dbearing = bearing + 315;
                }
            }
            if (dbearing > 360) {
                dbearing = dbearing - 360;
            }
        }

        return (int) dbearing;
    }

    private void sendAgentInfoToNearestAgents(AgentInfo agentInfo, List<Integer> nearestAgents) throws IOException {
        ACLMessage sendingMsg = new ACLMessage(ACLMessage.INFORM);
        for (int agentId : nearestAgents) {
            sendingMsg.addReceiver(new AID(String.valueOf(agentId), AID.ISLOCALNAME));
        }

        AgentToAgentMsg msg = new AgentToAgentMsg(
                agentInfo.id,
                agentInfo.azimuth,
                agentInfo.q
        );

        sendingMsg.setContentObject(msg);
        sendingMsg.setSender(this.getAgent().getAID());

        log.debug(LoggerWrapper.Action.SEND, sendingMsg);

        this.getAgent().send(sendingMsg);
    }

    private List<Integer> getNearestAgents(AgentInfo selfInfo) {
        return Arrays.asList(selfInfo.neighbours);
    }

    private void sendNewAzimuthToWebots(double newAzimuth) {
        publisher.publish(String.valueOf(newAzimuth));
    }

    private void clearAzimuths() {
        agentInfoById.clear();
    }
}
