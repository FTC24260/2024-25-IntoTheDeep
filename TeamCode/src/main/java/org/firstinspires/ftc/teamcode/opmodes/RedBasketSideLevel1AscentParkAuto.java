package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "RedBasketSideLevel1AscentParkAuto",  group = "Qualifiers" )
public class RedBasketSideLevel1AscentParkAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    int leftFrontPos;
    int rightFrontPos;
    int leftRearPos;
    int rightRearPos;
    private final int ticksPerInch = 188;
    private final double wheelbase = 16.5;
    private final double robotRotationCircumference = 73.4;
    private final double speed = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class,"leftRear");
        rightRear = hardwareMap.get(DcMotor.class,"rightRear");

        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFrontPos = 0;
        rightFrontPos = 0;
        leftRearPos = 0;
        rightRearPos = 0;

        runtime.reset();

        waitForStart();

        while (opModeIsActive()) {
            strafeLeft(12);
            driveForward(24);
            turnLeft(90);


        }
    }

    // Convert inches to ticks
    private int inchesToTicks(double inches) {
        return (int)(inches * ticksPerInch);
    }

    // Drive forward/backward by a specified distance in inches
    private void driveForward(double inches) {
        int ticks = inchesToTicks(inches);
        drive(ticks, ticks, ticks, ticks, speed);
    }

    private void driveBackward(double inches) {
        int ticks = inchesToTicks(inches);
        drive(-ticks, -ticks, -ticks, -ticks, speed);
    }

    // Strafe left/right by a specified distance in inches
    private void strafeRight(double inches) {
        // Positive inches = strafe right, negative = strafe left
        int ticks = inchesToTicks(inches);
        drive(ticks, -ticks, -ticks, ticks, speed);
    }

    private void strafeLeft(double inches) {
        // Positive inches = strafe right, negative = strafe left
        int ticks = inchesToTicks(inches);
        drive(-ticks, ticks, ticks, -ticks, speed);
    }

    // Turn by a specified distance in inches (measured at wheels)
    private void turnRight(double degrees) {
        // Calculate the distance the robot needs to travel for the given degrees
        double distance = (degrees/360.0) * robotRotationCircumference;

        // Convert this distance into ticks
        int ticks = (int) (distance * ticksPerInch);

        drive(ticks, -ticks, ticks, -ticks, speed);
    }

    private void turnLeft(double degrees) {
        double distance = (degrees/360.0) * robotRotationCircumference;

        // Convert this distance into ticks
        int ticks = (int) (distance * ticksPerInch);

        drive(-ticks, ticks, -ticks, ticks, speed);
    }


    // Original drive method modified to be private since we'll use the new methods above
    private void drive(int leftFrontTarget, int rightFrontTarget, int leftBackTarget, int rightBackTarget, double speed) {
        leftFrontPos += leftFrontTarget;
        rightFrontPos += rightFrontTarget;
        leftRearPos += leftBackTarget;
        rightRearPos += rightBackTarget;

        leftFront.setTargetPosition(leftFrontPos);
        rightFront.setTargetPosition(rightFrontPos);
        leftRear.setTargetPosition(leftRearPos);
        rightRear.setTargetPosition(rightRearPos);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftRear.setPower(speed);
        rightRear.setPower(speed);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            idle();
        }
    }
}