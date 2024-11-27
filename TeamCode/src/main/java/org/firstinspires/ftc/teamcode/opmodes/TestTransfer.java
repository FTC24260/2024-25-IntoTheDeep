package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestTransfer extends LinearOpMode {

    private DcMotorEx intakeMotor;
    private Servo intakeElbowR;
    private Servo intakeElbowL;

    private final double INTAKE_POWER = 0.33;
    private final int MOTOR_INTAKE_POSITION = 1000;
    private final int MOTOR_TOILET_POSITION = -1000;

    private final double ELBOW_UP = 1;
    private final double ELBOW_DOWN = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeElbowR = hardwareMap.get(Servo.class, "intakeElbowR");
        intakeElbowL = hardwareMap.get(Servo.class, "intakeElbowL");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                intakeMotor.setTargetPosition(100);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(0.5);
            }

            if (gamepad1.y) {
                goToIntake();
            }

            if (gamepad1.x) {
                goToToilet();
            }


        }

    }

    public void goToIntake() {
        //intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //intakeMotor.setPower(INTAKE_POWER);
        //intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeElbowR.setPosition(ELBOW_DOWN);
        intakeElbowL.setPosition(ELBOW_UP);
    }

    public void goToToilet() {
        //intakeMotor.setTargetPosition(MOTOR_TOILET_POSITION);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //intakeMotor.setPower(-INTAKE_POWER);
        //intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeElbowR.setPosition(ELBOW_UP);
        intakeElbowL.setPosition(ELBOW_DOWN);
    }

}
