package org.firstinspires.ftc.teamcode.commands;

public interface Command {
    void init();
    void execute();
    boolean isFinished();
    void end();
}
