package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.BasicTeleOp;

@Autonomous (name = "Observation2Specimens",  group = "Qualifiers" )
public class Observation2Specimens extends LinearOpMode {
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

    private final double ShoulderPositionTransfer = 0.7;
    private final double ShoulderPositionHighTransfer = 0.5;
    private final double ShoulderPositionSpecimen = 0.05;
    private final double ShoulderPositionRest = 0.6;
    private final double ShoulderPositionBasket = 0;


    //Intake Wrist
    private final double INTAKE_WRIST_90_CC = 0.5;
    private final double INTAKE_WRIST_UPSIDE_DOWN = 1;
    private final double INTAKE_WRIST_135_CC = 0.625;
    private final double INTAKE_WRIST_STRAIGHT = 0.16;
    private final double INTAKE_WRIST_45_CC = 0.365;

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
    private final double INTAKE_CLAW_LOOSELY_CLOSED_POSITION = 0.9;
    private final double INTAKE_CLAW_CLOSED_POSITION = 1;


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

    @Override
    public void runOpMode() {
        hwMapAndEncodersAndDriveDirections();
        runtime.reset();

        waitForStart();
        InitializedPosition();

        if (opModeIsActive()) {
            driveBackward(23,0.75);
            strafeRight(13,0.75);
            driveBackward(5,0.25);
            sleep(500);
            ShoulderBasket();
            driveForward(3,0.25);
            linearSlideR.setPower(1);
            linearSlideL.setPower(-1);
            sleep(600);
            linearSlidesStop();
            sleep(500);
            linearSlideR.setPower(-1);
            linearSlideL.setPower(1);
            sleep(600);
            linearSlidesStop();
            driveForward(1.5,0.25);
            openOuttakeClaw();
            ShoulderTransfer();
            driveForward(5,1);
            strafeLeft(50,1);
            intakeSpecimenMotor();
            openIntakeClaw();
            driveForward(16.5,0.5);
            intakeSpecimenServos();
            sleep(500);
            closeIntakeClaw();
            sleep(1000);
            goToRaisedIntake();
            goToTransfer();
            transferSample();
            driveBackward(5,0.5);
            strafeRight(50,1);
            driveBackward(14,1);
            ShoulderBasket();
            linearSlideR.setPower(1);
            linearSlideL.setPower(-1);
            sleep(600);
            linearSlidesStop();
            driveBackward(4,0.5);
            linearSlideR.setPower(-1);
            linearSlideL.setPower(1);
            sleep(600);
            linearSlidesStop();
            openOuttakeClaw();
            sleep(500);
            ShoulderTransfer();




            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);

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

    // Drive forward/backward by a specified distance in inches
    private void driveForward(double inches, double speed) {
        int ticks = inchesToTicks(inches);
        drive(ticks, ticks, ticks, ticks, speed);
    }

    private void driveBackward(double inches, double speed) {
        int ticks = inchesToTicks(inches);
        drive(-ticks, -ticks, -ticks, -ticks, speed);
    }

    // Strafe left/right by a specified distance in inches
    private void strafeRight(double inches, double speed) {
        // Positive inches = strafe right, negative = strafe left
        int ticks = inchesToTicks(inches);
        drive(ticks, -ticks, -ticks, ticks, speed);
    }

    private void strafeLeft(double inches, double speed) {
        // Positive inches = strafe right, negative = strafe left
        int ticks = inchesToTicks(inches);
        drive(-ticks, ticks, ticks, -ticks, speed);
    }

    // Turn by a specified distance in inches (measured at wheels)
    private void turnRight(double degrees, double speed) {
        double distance = (degrees/360.0) * robotRotationCircumference;
        int ticks = (int) (distance * ticksPerInch);
        drive(ticks, -ticks, ticks, -ticks, speed);
    }

    private void turnLeft(double degrees, double speed) {
        // Positive inches = turn right, negative = turn left
        double distance = (degrees/360.0) * robotRotationCircumference;
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
        intakeWristStraight();
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

    public void goToRaisedIntake() {
        intakeMotor.setTargetPosition(-MOTOR_SPECIMEN_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_UP_POWER);
    }


    public void goToPositioningFromIntake() {
        intakeElbowR.setPosition(R_ELBOW_LOW_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_LOW_POSITIONING);
        openIntakeClaw();
    }

    public void intakeSpecimenMotor() {
        intakeMotor.setTargetPosition(-MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_DOWN_POWER);
        intakeWristUpsideDown();
        openIntakeClaw();

    }

    public void intakeSpecimenServos() {
        intakeElbowR.setPosition(R_ELBOW_SPECIMEN_INTAKE);
        intakeElbowL.setPosition(L_ELBOW_SPECIMEN_INTAKE);

    }

    public void intakeWristUpsideDown() {
        intakeWrist.setPosition(INTAKE_WRIST_UPSIDE_DOWN);
    }

    public void intakeWristStraight() {
        intakeWrist.setPosition(INTAKE_WRIST_STRAIGHT);
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



    public void ShoulderSpecimen() {
        closeOuttakeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionSpecimen);
    }

    public void ShoulderBasket() {
        OuttakeShoulder.setPosition(ShoulderPositionBasket);
    }

    public void ShoulderTransfer() {
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

    }
}