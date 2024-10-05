package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Scheduler {
    private List<Command> commands = new ArrayList<>();

    public void addCommand(Command command) {
        commands.add(command);
    }

    public void run() {
        Iterator<Command> iterator = commands.iterator();
        while (iterator.hasNext()) {
            Command command = iterator.next();
            if (!command.isFinished()) {
                command.execute();
            } else {
                command.end();
                iterator.remove();
            }
        }
    }
}