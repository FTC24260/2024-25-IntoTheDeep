package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClaw extends SubsystemParent {
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSED_POSITION = 0.75;
    private ClawState clawState = ClawState.CLOSED;

    private enum ClawState {
        OPEN,
        CLOSED
    }

    private Servo claw;


    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_CLOSED_POSITION);
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POSITION);
        clawState = ClawState.OPEN;
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION);
        clawState = ClawState.CLOSED;
    }

    public void toggleClaw() {
        if (clawState == ClawState.CLOSED) {
            openClaw();
        } else {
            closeClaw();
        }
    }





}
