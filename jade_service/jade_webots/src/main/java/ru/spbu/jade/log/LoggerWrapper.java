package ru.spbu.jade.log;

import jade.lang.acl.ACLMessage;
import jade.util.leap.Iterator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class LoggerWrapper {

    private final Logger log;

    public LoggerWrapper(Class<?> clazz) {
        this.log = LoggerFactory.getLogger(clazz);
    }

    public void debug(Action action, ACLMessage msg) {
        log.debug(String.format(
                "'%s' FROM '%s' TO '%s' MSG '%s'",
                action,
                msg.getSender(),
                //TODO: check if logger can correctly convert set of values by iterator to String
                buildDebugMsgFromReceiverIterator(msg.getAllReceiver()),
                msg.getContent()
        ));
    }

    private String buildDebugMsgFromReceiverIterator(Iterator iter) {
        List<String> receivers = new ArrayList<>();
        while (iter.hasNext()) {
            Object receiver = iter.next();
            receivers.add(receiver.toString());
        }
        return receivers.stream().collect(Collectors.joining(",", "[", "]"));
    }

    public void error(String msg, Throwable exc, Object... args) {
        log.error(msg, args, exc);
    }

    public void error(String msg, Object... args) {
        log.error(msg, args);
    }

    public void warn(String msg, Object... args) {
        log.warn(msg, args);
    }

    public void debug(String msg, Object... args) {
        log.debug(msg, args);
    }

    public enum Action {
        SEND,
        RECEIVE
    }
}
