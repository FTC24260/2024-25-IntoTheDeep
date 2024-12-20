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
    private final double INTAKE_POWER = 0.3;
    private final int MOTOR_INTAKE_POSITION = 300;
    private final int MOTOR_TOILET_POSITION = 200;
    private final double RELBOW_TRANSFER = 0.25;
    private final double RELBOW_INTAKE = 1;
    private final double LELBOW_TRANSFER = 1;
    private final double LELBOW_INTAKE = 0.25;
    private final double ELBOW_INTAKE_POSITIONING = 0.25;
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSED_POSITION = 0.85;
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

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(CLAW_CLOSED_POSITION);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        openClaw();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                goToTransfer();
            }
            if (gamepad2.b){
                goToIntakePositioning();
            }
            if (gamepad2.y){
                goToIntake();
            }
            if (gamepad2.x){
                toggleClaw();
            }
        }
    }
    public void goToTransfer() {
        intakeElbowR.setPosition(RELBOW_TRANSFER);
        intakeElbowL.setPosition(LELBOW_TRANSFER);
        sleep(800);
        intakeMotor.setTargetPosition(MOTOR_TOILET_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep(2000);
        openClaw();

    }

    public void goToIntakePositioning() {
        intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_POWER);
        sleep(2000);
        intakeElbowR.setPosition(ELBOW_INTAKE_POSITIONING);
        intakeElbowL.setPosition(ELBOW_INTAKE_POSITIONING);
    }

    public void goToIntake() {
        intakeElbowR.setPosition(RELBOW_INTAKE);
        intakeElbowL.setPosition(LELBOW_INTAKE);
        sleep(1000);
        closeClaw();
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
            sleep(500);
        } else {
            closeClaw();
            sleep(500);
        }
    }
}
