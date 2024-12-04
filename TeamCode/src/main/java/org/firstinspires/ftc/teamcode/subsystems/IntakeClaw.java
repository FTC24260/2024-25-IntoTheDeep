package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeClaw extends SubsystemParent {
    private double CLAW_OPEN_POSITION = 0;
    private double CLAW_CLOSED_POSITION = 1;
    private ClawState clawState = ClawState.CLOSED;

    public enum ClawState {
        OPEN,
        CLOSED
    }

    private Servo claw;


    @Override
    public void init(HardwareMap hwMap) {
        claw = hwMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN_POSITION);
        telemetry.addData("ToggleClaw", getClawPosition().toString());
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POSITION);
        clawState = ClawState.OPEN;
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION);
        clawState = ClawState.CLOSED;
    }
    public ClawState getClawPosition(){
        return clawState;
    }

    public void toggleClaw() {
        if (clawState == ClawState.CLOSED) {
            openClaw();
        } else {
            closeClaw();
        }
    }





}
