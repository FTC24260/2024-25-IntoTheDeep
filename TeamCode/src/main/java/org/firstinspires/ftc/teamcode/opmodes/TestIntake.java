package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemParent;
@TeleOp

public class TestIntake extends LinearOpMode {

    private DcMotorEx intakeMotor;
    private Servo intakeElbowR;
    private Servo intakeElbowL;
    private Servo claw;
    private final double INTAKE_POWER = 0.08;
    private final int MOTOR_INTAKE_POSITION = 70;
    private final int MOTOR_TOILET_POSITION = -70;
    private final double ELBOW_UP = 0.87;
    private final double ELBOW_DOWN = 0.3;
            ;
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSED_POSITION = 0.75;
    enum ClawState {
        OPEN, CLOSED
    }
    private Intake.ClawState clawState = Intake.ClawState.CLOSED;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeElbowR = hardwareMap.get(Servo.class, "intakeElbowR");
        intakeElbowL = hardwareMap.get(Servo.class, "intakeElbowL");
        claw = hardwareMap.get(Servo.class, "claw");

        claw.setPosition(CLAW_CLOSED_POSITION);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.a) {
                goToToilet();
                openClaw();
            }
            if (gamepad2.b){
                goToIntake();
                openClaw();
            }
            if (gamepad2.x){
                toggleClaw();
            }
        }
    }
    public void goToToilet() {
        intakeMotor.setTargetPosition(MOTOR_TOILET_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_POWER);

        intakeElbowR.setPosition(ELBOW_UP);
        intakeElbowL.setPosition(ELBOW_DOWN);
    }

    public void goToIntake() {
        intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_POWER);

        intakeElbowR.setPosition(ELBOW_DOWN);
        intakeElbowL.setPosition(ELBOW_UP);
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POSITION);
        clawState = Intake.ClawState.OPEN;
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION);
        clawState = Intake.ClawState.CLOSED;
    }

    public void toggleClaw() {
        if (clawState == Intake.ClawState.CLOSED) {
            openClaw();
        } else {
            closeClaw();
        }
    }
}
