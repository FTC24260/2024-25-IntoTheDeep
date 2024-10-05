package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Basic: Teleop", group="Linear OpMode")

public class MotorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor ArmMotor = null;
    private DcMotor ClawAngle = null;
    private DcMotor linearSlide = null;
    private Servo servoClaw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("s", "forlan");
        telemetry.update();


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ClawAngle = hardwareMap.get(DcMotor.class, "ClawAngle");
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");


        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ClawAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        double speed = 0.5;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


                if (gamepad2.x) {
                    linearSlide.setPower(0.1);
                }
            }
        }
    }

