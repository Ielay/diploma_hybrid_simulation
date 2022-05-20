package ru.spbu.jade2.converter;

import org.json.JSONObject;
import ru.spbu.jade2.entity.AgentToAgentMsg;
import ru.spbu.jade2.entity.RobotToAgentMsg;
import ru.spbu.jade2.log.LoggerWrapper;

import java.awt.*;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class RobotInfoConverter {

    private static final LoggerWrapper log = new LoggerWrapper(AgentInfoConverter.class);

    private RobotInfoConverter() {
        //not constructable
    }

    public static RobotToAgentMsg toObjectFromProperties(String propertiesText) {
        try (InputStream in = new ByteArrayInputStream(propertiesText.getBytes())) {
            Properties props = new Properties();
            props.load(in);

            try {
                RobotToAgentMsg msg = new RobotToAgentMsg();
                msg.id = props.getProperty("id");
                msg.azimuth = Double.valueOf(props.getProperty("bearing"));
                String[] stringCoordinates = props.getProperty("neighbours").split(",");
                Integer[] convertedNeighbours = new Integer[stringCoordinates.length];
                for (int i = 0; i < stringCoordinates.length; ++i) {
                    convertedNeighbours[i] = Integer.valueOf(stringCoordinates[i]);
                }
                msg.neighbours = convertedNeighbours;
                msg.light = doubleArrayOf(props.getProperty("light"));
                msg.q = Double.valueOf(props.getProperty("q"));
                msg.d = Double.valueOf(props.getProperty("d"));

                return msg;
            } catch (Throwable exc) {
                log.error("Error building msg from 'Properties' obj = {}", props);
                throw exc;
            }
        } catch (IOException exc) {
            log.error("Error loading retrieved properties:\n" + propertiesText, exc);
            throw new RuntimeException(exc);
        }
    }

    public static Integer[] intArrayOf(String commaDelimitedValues) {
        if (commaDelimitedValues.isEmpty()) {
            return new Integer[0];
        }
        String[] stringValues = commaDelimitedValues.split(",");
        Integer[] intValues = new Integer[stringValues.length];
        for (int i = 0; i < stringValues.length; ++i) {
            intValues[i] = Integer.valueOf(stringValues[i]);
        }

        return intValues;
    }

    public static Double[] doubleArrayOf(String commaDelimitedValues) {
        String[] stringValues = commaDelimitedValues.split(",");
        Double[] doubleValues = new Double[stringValues.length];
        for (int i = 0; i < stringValues.length; ++i) {
            doubleValues[i] = Double.parseDouble(stringValues[i]);
        }

        return doubleValues;
    }
}
