package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Basic: Teleop", group="Linear OpMode")

public class BasicTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;


    @Override
    public void runOpMode() {
        telemetry.update();


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");


        //ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ClawAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        double speed = 0.5;

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
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            leftFront.setPower((drive + turn + strafe) * speed);
            rightFront.setPower((drive - turn - strafe) * speed);
            rightRear.setPower((drive - turn + strafe) * speed);
            leftRear.setPower((drive + turn - strafe) * speed);

            if (gamepad1.right_stick_y == 0) {
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (gamepad1.left_stick_x == 0) {
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Robot Speed
            if (gamepad1.a) {
                speed = 0.25;
            } else if (gamepad1.b) {
                speed = 0.5;
            } else if (gamepad1.y) {
                speed = 0.75;
            } else if (gamepad1.x) {
                speed = 1;
            } else if (gamepad1.right_trigger > 0.1) {
                speed = 0.25;
            } else if (gamepad1.right_trigger < 0.1) {
                speed = 0.5;
            } else if (gamepad1.left_trigger > 0.1) {
                speed = 1;
            } else if (gamepad1.left_trigger < 0.1) {
                speed = 0.5;
            }
        }
    }
}
