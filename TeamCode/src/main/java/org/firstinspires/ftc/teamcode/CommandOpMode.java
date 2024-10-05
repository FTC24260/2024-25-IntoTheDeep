package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.commands.Scheduler;

public abstract class CommandOpMode extends OpMode {
    protected Scheduler scheduler = new Scheduler();

    @Override
    public void init() {
        scheduler = new Scheduler();
        initialize();
    }

    @Override
    public void loop() {
        scheduler.run();
        execute();
    }

    protected abstract void initialize();
    protected abstract void execute();
}