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

public class BasicTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor chainMotor1 = null;
    private DcMotor chainMotor2 = null;
    private DcMotor linearSlide = null;
    private Servo servoClaw = null;

    @Override
    public void runOpMode() {
        telemetry.addData("s", "forlan");
        telemetry.update();


        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        chainMotor1 = hardwareMap.get(DcMotor.class, "chainMotor1");
        chainMotor2 = hardwareMap.get(DcMotor.class, "chainMotor2");
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        servoClaw = hardwareMap.get(Servo.class, "servoClaw");


        chainMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chainMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        double speed = 0.25;

        // chainMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to c
            // combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            leftFrontDrive.setPower((drive + turn + strafe) * speed);
            rightFrontDrive.setPower((drive - turn - strafe) * speed);
            rightBackDrive.setPower((drive - turn + strafe) * speed);
            leftBackDrive.setPower((drive + turn - strafe) * speed);

            if (gamepad1.right_stick_y == 0) {
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (gamepad1.left_stick_x == 0) {
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


            // chainMotor.setPower(gamepad2.right_stick_y/1.5);

            //chainMotor2.setPower((gamepad2.right_trigger-gamepad2.left_trigger)/2);
            //untested
            if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                linearSlide.setPower((gamepad2.left_stick_y));
                //linearSlide.setPower(0);
            } else {
                linearSlide.setPower(0);
                linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Robot Speed
            if (gamepad1.a == true) {
                speed = 0.25;
            } else if (gamepad1.b) {
                speed = 0.5;
            }/* else if (gamepad1.y) {
                speed = 0.75;
            } else if (gamepad1.x) {
                speed = 1;
            }*/

            //Intake in and out
            if (gamepad2.a) {
                chainMotor1.setPower(0.25);
            } else if (gamepad2.b) {
                chainMotor1.setPower(-0.25);
                chainMotor2.setTargetPosition(-900);
            } else {
                chainMotor1.setPower(0);
            }

            // intake up and down
            if (gamepad2.y) {
//500 is about parallel to the ground
                chainMotor2.setTargetPosition(-800);
                chainMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chainMotor2.setPower(0.5);
            } else if (gamepad2.x) {
                chainMotor2.setTargetPosition(-1160);
                chainMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chainMotor2.setPower(0.5);
            } else if (gamepad2.right_bumper) {
                chainMotor2.setTargetPosition(-1070);
                chainMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chainMotor2.setPower(0.5);
            } else if (gamepad2.left_bumper){
                chainMotor1.setPower(0.5);
            } else {
                chainMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                chainMotor2.setPower((gamepad2.right_trigger - gamepad2.left_trigger)/2);

            }
            if (gamepad2.start) {
                servoClaw.setPosition(1);

            } else {
                servoClaw.setPosition(0);
            }




        }

    }
}

