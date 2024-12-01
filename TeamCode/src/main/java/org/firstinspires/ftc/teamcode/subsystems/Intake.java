package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemParent;

@TeleOp
public class Intake extends SubsystemParent {
    private DcMotorEx intakeMotor;
    private Servo intakeElbowR;
    private Servo intakeElbowL;
    private Servo claw;

    private final double INTAKE_POWER = 0.33;
    private final int MOTOR_INTAKE_POSITION = 1000;
    private final int MOTOR_TOILET_POSITION = -1000;
    private final double ELBOW_UP = 0.25;
    private final double ELBOW_DOWN = 0;
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSED_POSITION = 0.75;

    public enum ClawState {
        OPEN, CLOSED
    }

    private ClawState clawState = ClawState.CLOSED;

    @Override
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeElbowR = hwMap.get(Servo.class, "intakeElbowR");
        intakeElbowL = hwMap.get(Servo.class, "intakeElbowL");
        claw = hwMap.get(Servo.class, "claw");

        claw.setPosition(CLAW_CLOSED_POSITION);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToToilet() {
        intakeMotor.setTargetPosition(MOTOR_TOILET_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_POWER);

        intakeElbowR.setPosition(ELBOW_DOWN);
        intakeElbowL.setPosition(ELBOW_DOWN);
    }

    public void goToIntake() {
        intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_POWER);

        intakeElbowR.setPosition(ELBOW_UP);
        intakeElbowL.setPosition(ELBOW_UP);
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