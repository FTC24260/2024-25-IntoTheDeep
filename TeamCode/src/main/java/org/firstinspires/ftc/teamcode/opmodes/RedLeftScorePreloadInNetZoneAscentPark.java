package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "RedBasketSideLevel1AscentParkAuto",  group = "Qualifiers" )
public class RedLeftScorePreloadInNetZoneAscentPark extends LinearOpMode {
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

    int leftFrontPos;
    int rightFrontPos;
    int leftRearPos;
    int rightRearPos;
    private final int ticksPerInch = 188;
    private final double wheelbase = 16.5;
    private final double robotRotationCircumference = 73.4;
    private final double speed = 0.5;
    private final double POSITIONING_SPEED = 0.2;

    private final double ShoulderPositionTransfer = 0.4;
    private final double ShoulderPositionSpecimen = 1;
    private final double ShoulderPositionRest = 0.05;
    private final double ShoulderPositionBasket = 0.85;

    //Linear Slides
    private final double linearSlidesPower = 1;
    private final double linearSlidesBufferZone = 0.1;

    //Outtake Claw
    private final double OUTTAKE_CLAW_DEFAULT_OPEN_POSITION = 0.1;
    private final double OUTTAKE_CLAW_CLOSED_POSITION = 0.39;

    //Intake Elbows
    private final double L_ELBOW_INTAKE = 0.9;
    private final double R_ELBOW_INTAKE = 0.4;
    private final double L_ELBOW_HIGH_POSITIONING = 0.68;
    private final double R_ELBOW_HIGH_POSITIONING = 0.65;
    private final double L_ELBOW_LOW_POSITIONING = 0.8;
    private final double R_ELBOW_LOW_POSITIONING = 0.53;
    private final double L_ELBOW_TRANSFER = 0.29;
    private final double R_ELBOW_TRANSFER = 0.91;
    private final double R_ELBOW_FULLY_BACK = 1;
    private final double L_ELBOW_FULLY_BACK = 0.2;

    //Intake Claw
    private final double INTAKE_CLAW_OPEN_POSITION = 0;
    private final double INTAKE_CLAW_LOOSELY_CLOSED_POSITION = 0.31;
    private final double INTAKE_CLAW_CLOSED_POSITION = 0.33;

    //Intake Shoulder Motor
    private final double INTAKE_UP_POWER = 0.7;
    private final double INTAKE_DOWN_POWER = 0.5;
    private final int MOTOR_INTAKE_POSITION = 975;
    private final int MOTOR_TRANSFER_POSITION = 550;
    private final int MOTOR_FULLY_BACK_POSITION = 0;



    private enum ClawState {
        OPEN, CLOSED
    }

    private enum IntakeState {
        POSITIONING, INTAKE, TRANSFER
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hwMapAndEncodersAndDriveDirections();

        runtime.reset();

        waitForStart();

        while (opModeIsActive()) {
            driveForward(12);
            strafeLeft(12);
            turnLeft(135);
            goToPositioning();
            goToIntakeFromPositioning();
            goToFullyBack();
            strafeRight(12);
            driveForward(36);
            turnLeft(90);
            ShoulderBasket();
            driveBackward(10.25);


        }
    }

    // Convert inches to ticks
    private int inchesToTicks(double inches) {
        return (int) (inches * ticksPerInch);
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
        double distance = (degrees / 360.0) * robotRotationCircumference;

        // Convert this distance into ticks
        int ticks = (int) (distance * ticksPerInch);

        drive(ticks, -ticks, ticks, -ticks, speed);
    }

    private void turnLeft(double degrees) {
        double distance = (degrees / 360.0) * robotRotationCircumference;

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

    public void goToTransfer() {
        intakeElbowR.setPosition(R_ELBOW_TRANSFER);
        intakeElbowL.setPosition(L_ELBOW_TRANSFER);
        closeLooselyIntakeClaw();
        sleep(1000);
        intakeMotor.setTargetPosition(MOTOR_TRANSFER_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_UP_POWER);
    }

    public void goToPositioning() {
        intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_DOWN_POWER);
        openOuttakeClaw();
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
    }

    private void linearSlidesDown() {
        linearSlideL.setPower(-linearSlidesPower);
        linearSlideR.setPower(linearSlidesPower);
    }

    private void linearSlidesStop() {
        linearSlideL.setPower(0);
        linearSlideR.setPower(0);
    }

    public void ShoulderSpecimen() {
        closeOuttakeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionSpecimen);
    }

    public void ShoulderBasket() {
        closeOuttakeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionBasket);
    }

    public void ShoulderTransfer() {
        closeOuttakeClaw();
        sleep(500);
        OuttakeShoulder.setPosition(ShoulderPositionTransfer);
    }

    public void ShoulderRest() {
        closeOuttakeClaw();
        sleep(200);
        OuttakeShoulder.setPosition(ShoulderPositionRest);
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

        linearSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linearSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        OuttakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        leftFrontPos = 0;
        rightFrontPos = 0;
        leftRearPos = 0;
        rightRearPos = 0;
    }

    public void InitializedPosition() {
        ShoulderRest();
        openIntakeClaw();

    }
}
