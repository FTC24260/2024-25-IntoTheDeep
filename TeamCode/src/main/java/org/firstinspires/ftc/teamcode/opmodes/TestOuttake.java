package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class TestOuttake extends LinearOpMode {
    private DcMotorEx linearSlideL;
    private DcMotorEx linearSlideR;
    private Servo OuttakeShoulder;
    private Servo OuttakeClaw;
    private final double ShoulderPositionSpecimen = 0.8;
    private final double ShoulderPositionTransfer = 0.4;
    private final double ShoulderPositionBasket = 0;
    private final double linearSlidesPower = 0.75;
    private final double CLAW_FULL_OPEN_POSITION = 0;
    private final double CLAW_DEFAULT_OPEN_POSITION = 0.1;
    private final double CLAW_CLOSED_POSITION = 0.4;
    private final int R_linearSlidesMax = 1000;
    private final int L_linearSlidesMin = 0;
    private final double linearSlidesBufferZone = 0.1;

    private enum ClawState {
        OPEN, CLOSED
    }
    private Intake.ClawState clawState = Intake.ClawState.CLOSED;

    @Override
    public void runOpMode() throws InterruptedException {
        linearSlideL = hardwareMap.get(DcMotorEx.class, "linearSlideL");
        linearSlideR = hardwareMap.get(DcMotorEx.class, "linearSlideR");
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeShoulder = hardwareMap.get(Servo.class, "OuttakeShoulder");
        linearSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_trigger > linearSlidesBufferZone) {
                //ShoulderTransfer();
                //sleep(3000);
                linearSlidesUp();

            } else if (gamepad2.left_trigger > linearSlidesBufferZone) {
                linearSlidesDown();
                //sleep(3000);
                //ShoulderBasket();

            } else if (gamepad2.right_stick_y > linearSlidesBufferZone && gamepad2.right_stick_button) {
                linearSlidesUp();
                sleep(3000);
                ShoulderSpecimen();
            }
            else {
                linearSlidesStop();
            }
            if (gamepad2.right_bumper) {
                ShoulderSpecimen();
            }

            if (gamepad2.left_bumper) {
                ShoulderBasket();
            }
            if (gamepad2.x) {
                openClaw();

            }
            if (gamepad2.y) {
                closeClaw();

            }
        }
    }

    private void linearSlidesUp() {
        // Move up by setting a negative target position
        linearSlideL.setPower(linearSlidesPower);
        linearSlideR.setPower(-linearSlidesPower);
    }

    private void linearSlidesDown() {
        // Move down by setting a positive target position
        linearSlideL.setPower(-linearSlidesPower);
        linearSlideR.setPower(linearSlidesPower);
    }
    private void linearSlidesStop() {
        linearSlideL.setPower(0);
        linearSlideR.setPower(0);
    }
    public void ShoulderSpecimen() {
        closeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionSpecimen);
    }
    public void ShoulderBasket() {
        closeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionBasket);
    }
    public void ShoulderTransfer() {
        closeClaw();
        sleep(500);
        OuttakeShoulder.setPosition(ShoulderPositionTransfer);
    }
    public void openClaw() {
        OuttakeClaw.setPosition(CLAW_DEFAULT_OPEN_POSITION);
        clawState = Intake.ClawState.OPEN;
    }

    public void closeClaw() {
        OuttakeClaw.setPosition(CLAW_CLOSED_POSITION);
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