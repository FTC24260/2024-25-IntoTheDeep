package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.BasicTeleOp;

@Autonomous (name = "TestDirectionsAuto",  group = "Qualifiers" )
public class TestDirectionsAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotorEx intakeMotor;
    private DcMotorEx linearSlideL;
    private DcMotorEx linearSlideR;
    private Servo intakeElbowR;
    private Servo intakeElbowL;
    private Servo claw;
    private Servo OuttakeShoulder;
    private Servo OuttakeClaw;
    private Servo intakeWrist;

    int leftFrontPos;
    int rightFrontPos;
    int leftRearPos;
    int rightRearPos;
    private final double ticksPerInch = 43.5;
    private final double wheelbase = 16.5;
    private final double robotRotationCircumference = 73.4;
    private final double speed = 0.5;
    private final double POSITIONING_SPEED = 0.2;

    //Outtake Shoulder

    private final double ShoulderPositionTransfer = 0.7;
    private final double ShoulderPositionHighTransfer = 0.5;
    private final double ShoulderPositionSpecimen = 0.05;
    private final double ShoulderPositionRest = 0.6;
    private final double ShoulderPositionBasket = 0;

    //Linear Slides
    private final double linearSlidesPower = 1;
    private final double linearSlidesBufferZone = 0.1;
    private final int LINEAR_SLIDES_MAX_POSITION = 2700;
    private final int LINEAR_SLIDES_MIN_POSITION = 0;

    //Outtake Claw
    private final double OUTTAKE_CLAW_DEFAULT_OPEN_POSITION = 0.4;
    private final double OUTTAKE_CLAW_CLOSED_POSITION = 0.79;
    private final double OUTTAKE_CLAW_LOOSELY_CLOSED_POSITION = 0.6;


    //Intake Elbows
    private final double L_ELBOW_INTAKE = 1;
    private final double R_ELBOW_INTAKE = 0.3;
    private final double L_ELBOW_HIGH_POSITIONING = 0.68;
    private final double R_ELBOW_HIGH_POSITIONING = 0.65;
    private final double L_ELBOW_LOW_POSITIONING = 0.67;
    private final double R_ELBOW_LOW_POSITIONING = 0.66;
    private final double L_ELBOW_TRANSFER = 0.33;
    private final double R_ELBOW_TRANSFER = 0.92;
    private final double R_ELBOW_FULLY_BACK = 1;
    private final double L_ELBOW_FULLY_BACK = 0.2;
    private final double R_ELBOW_SPECIMEN_INTAKE = 0.82;
    private final double L_ELBOW_SPECIMEN_INTAKE = 0.51;

    //Intake Claw
    private final double INTAKE_CLAW_OPEN_POSITION = 0.5;
    private final double INTAKE_CLAW_LOOSELY_CLOSED_POSITION = 0.68;
    private final double INTAKE_CLAW_CLOSED_POSITION = 1;

    //Intake Wrist
    private final double INTAKE_WRIST_90_CC = 0.44;
    private final double INTAKE_WRIST_UPSIDE_DOWN = 1;
    private final double INTAKE_WRIST_135_CC = 0.6;
    private final double INTAKE_WRIST_STRAIGHT = 0.14;
    private final double INTAKE_WRIST_45_CC = 0.32;
    private final double INTAKE_WRIST_45_C = 0;

    //Intake Shoulder Motor
    private final double INTAKE_UP_POWER = 1;
    private final double INTAKE_DOWN_POWER = 1;
    private final int MOTOR_INTAKE_POSITION = 1050;
    private final int MOTOR_SPECIMEN_INTAKE_POSITION = 800;
    private final int MOTOR_TRANSFER_POSITION = 450;
    private final int MOTOR_FULLY_BACK_POSITION = 0;


    private enum ClawState {
        OPEN, CLOSED
    }

    private enum IntakeState {
        POSITIONING, INTAKE, TRANSFER
    }
    private enum IntakeWristState {
        STRAIGHT, DIAGONALCC, DIAGONALC, HORIZONTALC, HORIZONTALCC, UPSIDEDOWN
    }
    @Override
    public void runOpMode() {
        hwMapAndEncodersAndDriveDirections();
        runtime.reset();

        waitForStart();
        InitializedPosition();

        if (opModeIsActive()) {
            driveForwardAccelDecel(10,0.5);
            waitForMotors();
            strafeRightAccelDecel(35,0.5);
            waitForMotors();
            driveBackwardAccelDecel(10,0.5);
            waitForMotors();

            while (opModeIsActive()) {
                telemetry.addData("Status", "Holding Position");
                telemetry.addData("Left Front Position", leftFront.getCurrentPosition());
                telemetry.addData("Right Front Position", rightFront.getCurrentPosition());
                telemetry.addData("Left Rear Position", leftRear.getCurrentPosition());
                telemetry.addData("Right Rear Position", rightRear.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    // Convert inches to ticks
    private int inchesToTicks(double inches) {
        return (int)(inches * ticksPerInch);
    }

    private void waitForMotors() {
        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftRear.isBusy() || rightRear.isBusy())) {
            telemetry.addData("Status", "Moving...");
            telemetry.addData("Left Front", leftFront.getCurrentPosition());
            telemetry.addData("Right Front", rightFront.getCurrentPosition());
            telemetry.addData("Left Rear", leftRear.getCurrentPosition());
            telemetry.addData("Right Rear", rightRear.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

    // Drive forward/backward with acceleration/deceleration
    private void driveForwardAccelDecel(double inches, double maxSpeed) {
        int ticks = inchesToTicks(inches);
        driveAccelDecel(ticks, ticks, ticks, ticks, maxSpeed);
    }

    private void driveBackwardAccelDecel(double inches, double maxSpeed) {
        int ticks = inchesToTicks(inches);
        driveAccelDecel(-ticks, -ticks, -ticks, -ticks, maxSpeed);
    }

    // Strafe with acceleration/deceleration
    private void strafeRightAccelDecel(double inches, double maxSpeed) {
        int ticks = inchesToTicks(inches);
        driveAccelDecel(ticks, -ticks, -ticks, ticks, maxSpeed);
    }

    private void strafeLeftAccelDecel(double inches, double maxSpeed) {
        int ticks = inchesToTicks(inches);
        driveAccelDecel(-ticks, ticks, ticks, -ticks, maxSpeed);
    }

    // Turn with acceleration/deceleration
    private void turnRightAccelDecel(double degrees, double maxSpeed) {
        double distance = (degrees/360.0) * robotRotationCircumference;
        int ticks = (int) (distance * ticksPerInch);
        driveAccelDecel(ticks, -ticks, ticks, -ticks, maxSpeed);
    }

    private void turnLeftAccelDecel(double degrees, double maxSpeed) {
        double distance = (degrees/360.0) * robotRotationCircumference;
        int ticks = (int) (distance * ticksPerInch);
        driveAccelDecel(-ticks, ticks, -ticks, ticks, maxSpeed);
    }

    // Modified driveAccelDecel to accept individual motor ticks
    private void driveAccelDecel(int leftFrontTicks, int rightFrontTicks, int leftRearTicks, int rightRearTicks, double maxSpeed) {
        // Set target positions
        leftFront.setTargetPosition(leftFrontPos + leftFrontTicks);
        rightFront.setTargetPosition(rightFrontPos + rightFrontTicks);
        leftRear.setTargetPosition(leftRearPos + leftRearTicks);
        rightRear.setTargetPosition(rightRearPos + rightRearTicks);

        // Switch to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start at zero speed
        double currentSpeed = 0;

        // Calculate total movement distance (use average of absolute ticks)
        int totalTicks = (Math.abs(leftFrontTicks) + Math.abs(rightFrontTicks) +
                Math.abs(leftRearTicks) + Math.abs(rightRearTicks)) / 4;

        // Calculate midpoint for acceleration/deceleration
        int midpoint = totalTicks / 2;

        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() ||
                        leftRear.isBusy() || rightRear.isBusy())) {

            // Calculate progress (average of all motors)
            int currentProgress = (Math.abs(leftFront.getCurrentPosition() - leftFrontPos) +
                    Math.abs(rightFront.getCurrentPosition() - rightFrontPos) +
                    Math.abs(leftRear.getCurrentPosition() - leftRearPos) +
                    Math.abs(rightRear.getCurrentPosition() - rightRearPos)) / 4;

            // Acceleration phase until halfway
            if (currentProgress < midpoint) {
                currentSpeed = Math.min(currentSpeed + 0.01, maxSpeed);
            }
            // Deceleration phase after halfway
            else {
                currentSpeed = Math.max(currentSpeed - 0.01, 0);
            }

            // Apply power to all motors
            leftFront.setPower(currentSpeed);
            rightFront.setPower(currentSpeed);
            leftRear.setPower(currentSpeed);
            rightRear.setPower(currentSpeed);

            sleep(20); // Small delay for smooth acceleration/deceleration

            telemetry.addData("Current Speed", currentSpeed);
            telemetry.addData("Progress", "%d / %d", currentProgress, totalTicks);
            telemetry.update();
        }

        // Update stored positions
        leftFrontPos = leftFront.getCurrentPosition();
        rightFrontPos = rightFront.getCurrentPosition();
        leftRearPos = leftRear.getCurrentPosition();
        rightRearPos = rightRear.getCurrentPosition();

        // Stop all motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
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
    public void goToTransfer() {
        intakeElbowR.setPosition(R_ELBOW_TRANSFER);
        intakeElbowL.setPosition(L_ELBOW_TRANSFER);
        closeLooselyIntakeClaw();
        sleep(1000);
        intakeMotor.setTargetPosition(-MOTOR_TRANSFER_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_UP_POWER);
    }

    public void goToPositioning() {
        intakeMotor.setTargetPosition(-MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_DOWN_POWER);
        sleep(1000);
        intakeElbowR.setPosition(R_ELBOW_HIGH_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_HIGH_POSITIONING);
        sleep(500);
        intakeElbowR.setPosition(R_ELBOW_LOW_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_LOW_POSITIONING);
    }

    public void goToIntakeFromPositioning() {
        intakeElbowR.setPosition(R_ELBOW_INTAKE);
        intakeElbowL.setPosition(L_ELBOW_INTAKE);
        sleep(1000);
        openIntakeClaw();
    }

    public void goToPositioningFromIntake() {
        intakeElbowR.setPosition(R_ELBOW_LOW_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_LOW_POSITIONING);
        openIntakeClaw();
    }

    public void goToFullyBack() {
        intakeElbowR.setPosition(R_ELBOW_FULLY_BACK);
        intakeElbowL.setPosition(L_ELBOW_FULLY_BACK);
        sleep(500);
        intakeMotor.setTargetPosition(MOTOR_FULLY_BACK_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_UP_POWER);
    }

    public void openIntakeClaw() {
        claw.setPosition(INTAKE_CLAW_OPEN_POSITION);
    }

    public void closeIntakeClaw() {
        claw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
    }

    public void closeLooselyIntakeClaw() {
        claw.setPosition(INTAKE_CLAW_LOOSELY_CLOSED_POSITION);
    }

    private void linearSlidesUp() {
        linearSlideL.setPower(linearSlidesPower);
        linearSlideR.setPower(-linearSlidesPower);
        sleep(500);
        linearSlidesStop();
    }

    private void linearSlidesDown() {
        linearSlideL.setPower(linearSlidesPower);
        linearSlideR.setPower(-linearSlidesPower);
        sleep(500);
        linearSlidesStop();
    }

    private void linearSlidesStop() {
        linearSlideL.setPower(0);
        linearSlideR.setPower(0);
    }

    public void intakeWristStraight() {
        intakeWrist.setPosition(INTAKE_WRIST_STRAIGHT);
    }

    public void intakeWrist45CC() {
        intakeWrist.setPosition(INTAKE_WRIST_45_CC);
    }

    public void intakeWrist90CC() {
        intakeWrist.setPosition(INTAKE_WRIST_90_CC);
    }

    public void intakeWrist135CC() {
        intakeWrist.setPosition(INTAKE_WRIST_135_CC);
    }

    public void intakeWrist45C() {
        intakeWrist.setPosition(INTAKE_WRIST_45_C);
    }

    public void intakeWristUpsideDown() {
        intakeWrist.setPosition(INTAKE_WRIST_UPSIDE_DOWN);
    }

    public void ShoulderSpecimen() {
        closeOuttakeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionSpecimen);
    }

    public void ShoulderBasket() {
        OuttakeShoulder.setPosition(ShoulderPositionBasket);
        sleep(1200);
        openOuttakeClaw();
    }

    public void ShoulderTransfer() {
        closeOuttakeClaw();
        sleep(500);
        OuttakeShoulder.setPosition(ShoulderPositionTransfer);
    }
    public void openOuttakeClaw() {
        OuttakeClaw.setPosition(OUTTAKE_CLAW_DEFAULT_OPEN_POSITION);
    }

    public void closeOuttakeClaw() {
        OuttakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
    }

    public void transferSample() {
        closeOuttakeClaw();
        sleep(50);
        openIntakeClaw();

    }

    public void linearSlidesTarget(int linearSlideHeight) {
        linearSlideL.setTargetPosition(-linearSlideHeight);
        linearSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideL.setPower(-linearSlidesPower);

        linearSlideR.setTargetPosition(linearSlideHeight);
        linearSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideR.setPower(-linearSlidesPower);

    }

    public void goToIntake() {
        goToPositioning();
        goToIntakeFromPositioning();
    }

    public void hwMapAndEncodersAndDriveDirections() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeElbowR = hardwareMap.get(Servo.class, "intakeElbowR");
        intakeElbowL = hardwareMap.get(Servo.class, "intakeElbowL");
        claw = hardwareMap.get(Servo.class, "claw");
        linearSlideL = hardwareMap.get(DcMotorEx.class, "linearSlideL");
        linearSlideR = hardwareMap.get(DcMotorEx.class, "linearSlideR");
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeShoulder = hardwareMap.get(Servo.class, "OuttakeShoulder");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");

        linearSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        OuttakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        leftFrontPos = 0;
        rightFrontPos = 0;
        leftRearPos = 0;
        rightRearPos = 0;
    }

    public void InitializedPosition() {
        ShoulderTransfer();
        closeOuttakeClaw();
        intakeWristStraight();

    }
}