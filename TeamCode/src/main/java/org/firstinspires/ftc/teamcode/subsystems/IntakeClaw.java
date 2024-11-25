package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClaw extends SubsystemParent {
    private double clawOpenPos = 0.25;
    private double clawClosedPos = 0.0;
    private ClawState clawState = ClawState.CLOSED;

    private enum ClawState {
        OPEN,
        CLOSED
    }

    private Servo claw;


    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
    }

    public void openClaw() {
        claw.setPosition(clawOpenPos);
        clawState = ClawState.OPEN;
    }

    public void closeClaw() {
        claw.setPosition(clawClosedPos);
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
