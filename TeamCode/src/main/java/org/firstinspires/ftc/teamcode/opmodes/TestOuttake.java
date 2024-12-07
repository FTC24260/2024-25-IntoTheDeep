package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestOuttake extends LinearOpMode {
    private DcMotorEx linearSlideL;
    private DcMotorEx linearSlideR;
    private final int linearSlidesPosition = 3900;
    private Servo OuttakeShoulder;
    private Servo OuttakeClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        linearSlideL = hardwareMap.get(DcMotorEx.class, "linearSlideL");
        linearSlideR = hardwareMap.get(DcMotorEx.class, "linearSlideR");
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeShoulder = hardwareMap.get(Servo.class, "OuttakeShoulder");
        linearSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_trigger > 0.1) {
                linearSlidesUp();
            }
            if (gamepad2.left_trigger > 0.1) {
                linearSlidesDown();
            }
            if (gamepad2.right_bumper){
                OuttakeShoulder.setPosition(1);
                OuttakeClaw.setPosition(0.4);
            }

            if (gamepad2.left_bumper){
                OuttakeShoulder.setPosition(0);
                OuttakeClaw.setPosition(1);
            }
        }
    }

    private void linearSlidesUp() {
        // Move up by setting a negative target position
        linearSlideL.setTargetPosition(-linearSlidesPosition);
        linearSlideR.setTargetPosition(linearSlidesPosition);
        linearSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideL.setPower(0.5);
        linearSlideR.setPower(0.5);
    }

    private void linearSlidesDown() {
        // Move down by setting a positive target position
        linearSlideL.setTargetPosition(linearSlidesPosition);
        linearSlideR.setTargetPosition(-linearSlidesPosition);
        linearSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideL.setPower(0.5);
        linearSlideR.setPower(0.5);
    }
}