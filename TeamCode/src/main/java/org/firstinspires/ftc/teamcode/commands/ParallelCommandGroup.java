package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.List;

public class ParallelCommandGroup implements Command {
    private List<Command> commands = new ArrayList<>();

    public ParallelCommandGroup(Command... commands) {
        for (Command command : commands) {
            this.commands.add(command);
        }
    }

    @Override
    public void init() {
        for (Command command : commands) {
            command.init();
        }
    }

    @Override
    public void execute() {
        for (Command command : commands) {
            if (!command.isFinished()) {
                command.execute();
            }
        }
    }

    @Override
    public boolean isFinished() {
        for (Command command : commands) {
            if (!command.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void end() {
        for (Command command : commands) {
            command.end();
        }
    }
}