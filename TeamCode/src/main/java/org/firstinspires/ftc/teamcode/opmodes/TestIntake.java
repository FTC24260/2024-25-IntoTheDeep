package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp

public class TestIntake extends LinearOpMode {

    private DcMotorEx intakeMotor;
    private Servo intakeElbowR;
    private Servo intakeElbowL;
    private Servo claw; 
    private final double INTAKE_POWER = 0.3;
    private final int MOTOR_INTAKE_POSITION = 300;
    private final int MOTOR_TRANSFER_POSITION = 200;
    private final double R_ELBOW_TRANSFER = 0.25;
    private final double R_ELBOW_INTAKE = 1;
    private final double L_ELBOW_TRANSFER = 0.25;
    private final double L_ELBOW_INTAKE = 1;
    private final double R_ELBOW_INTAKE_POSITIONING = 0.75;
    private final double L_ELBOW_INTAKE_POSITIONING = 0.75;
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSED_POSITION = 0.85;

    enum ClawState {
        OPEN, CLOSED
    }
    private Intake.ClawState clawState = Intake.ClawState.CLOSED;

    private enum IntakeState {
        POSITIONING, INTAKE, TRANSFER
    }
    private Intake.IntakeState intakeState = Intake.IntakeState.TRANSFER;

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
            if (gamepad2.right_bumper){
                toggleIntake();
            }
        }
    }

    public void goToTransfer() {
        intakeElbowR.setPosition(R_ELBOW_TRANSFER);
        intakeElbowL.setPosition(L_ELBOW_TRANSFER);
        sleep(800);
        intakeMotor.setTargetPosition(MOTOR_TRANSFER_POSITION);
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
        intakeElbowR.setPosition(R_ELBOW_INTAKE_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_INTAKE_POSITIONING);
    }

    public void goToIntake() {
        intakeElbowR.setPosition(R_ELBOW_INTAKE);
        intakeElbowL.setPosition(L_ELBOW_INTAKE);
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

    public void toggleIntake() {
        if (intakeState == Intake.IntakeState.TRANSFER) {
            intakeElbowR.setPosition(R_ELBOW_INTAKE_POSITIONING);
            intakeElbowL.setPosition(L_ELBOW_INTAKE_POSITIONING);

        } else if (intakeState == Intake.IntakeState.POSITIONING) {
            intakeElbowR.setPosition(R_ELBOW_INTAKE);
            intakeElbowL.setPosition(L_ELBOW_INTAKE);

        } else {
            intakeElbowR.setPosition(R_ELBOW_INTAKE_POSITIONING);
            intakeElbowL.setPosition(L_ELBOW_INTAKE_POSITIONING);
        }
    }
}
