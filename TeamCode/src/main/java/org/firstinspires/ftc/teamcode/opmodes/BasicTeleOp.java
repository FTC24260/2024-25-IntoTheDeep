package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Date;

@TeleOp(name="Basic: Teleop", group="Linear OpMode")
public class BasicTeleOp extends LinearOpMode {
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
    private Servo intakeWrist;
    private Servo OuttakeShoulder;
    private Servo OuttakeClaw;

    //Outtake Shoulder
    private final double ShoulderPositionTransfer = 0.53;
    private final double ShoulderPositionSpecimen = 1;
    private final double ShoulderPositionRest = 0.6;
    private final double ShoulderPositionBasket = 0.1;

    //Linear Slides
    private final double linearSlidesPower = 1;
    private final double linearSlidesBufferZone = 0.1;
    private final int LINEAR_SLIDES_MAX_POSITION = 2700;
    private final int LINEAR_SLIDES_MIN_POSITION = 0;

    //Outtake Claw
    private final double OUTTAKE_CLAW_DEFAULT_OPEN_POSITION = 0.1;
    private final double OUTTAKE_CLAW_CLOSED_POSITION = 0.65;

    //Intake Elbows
    private final double L_ELBOW_INTAKE = 1;
    private final double R_ELBOW_INTAKE = 0.3;
    private final double L_ELBOW_HIGH_POSITIONING = 0.68;
    private final double R_ELBOW_HIGH_POSITIONING = 0.65;
    private final double L_ELBOW_LOW_POSITIONING = 0.67;
    private final double R_ELBOW_LOW_POSITIONING = 0.66;
    private final double L_ELBOW_TRANSFER = 0.3;
    private final double R_ELBOW_TRANSFER = 0.95;
    private final double R_ELBOW_FULLY_BACK = 1;
    private final double L_ELBOW_FULLY_BACK = 0.2;
    private final double R_ELBOW_SPECIMEN_INTAKE = 0.78;
    private final double L_ELBOW_SPECIMEN_INTAKE = 0.55;

    //Intake Claw
    private final double INTAKE_CLAW_OPEN_POSITION = 0;
    private final double INTAKE_CLAW_LOOSELY_CLOSED_POSITION = 0.23;
    private final double INTAKE_CLAW_CLOSED_POSITION = 0.25;

    //Intake Wrist
    private final double INTAKE_WRIST_STRAIGHT = 0.48;
    private final double INTAKE_WRIST_90_CC = 0.82;
    private final double INTAKE_WRIST_45_CC = 0.625;
    private final double INTAKE_WRIST_90_C = 0.16;
    private final double INTAKE_WRIST_45_C = 0.365;

    //Intake Shoulder Motor
    private final double INTAKE_UP_POWER = 1;
    private final double INTAKE_DOWN_POWER = 1;
    private final int MOTOR_INTAKE_POSITION = 975;
    private final int MOTOR_SPECIMEN_INTAKE_POSITION = 1150;
    private final int MOTOR_TRANSFER_POSITION = 500;
    private final int MOTOR_FULLY_BACK_POSITION = 0;

    //Drive Speeds
    private double speed = 0.5;
    private final double POSITIONING_SPEED = 0.25;


    private enum ClawState {
        OPEN, CLOSED
    }

    private ClawState clawState = ClawState.CLOSED;

    private enum IntakeState {
        POSITIONING, INTAKE, TRANSFER
    }

    private IntakeState intakeState = IntakeState.TRANSFER;

    private enum OuttakeShoulderState {
        TRANSFER, BASKET, SPECIMEN
    }

    private OuttakeShoulderState outtakeShoulderState = OuttakeShoulderState.TRANSFER;

    private enum IntakeWristState {
        STRAIGHT, DIAGONALCC, DIAGONALC, HORIZONTALC, HORIZONTALCC
    }
    private IntakeWristState intakeWristState = IntakeWristState.STRAIGHT;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hwMapAndEncodersAndDriveDirections();


        waitForStart();

        InitializedPosition();

        runtime.reset();

        while (opModeIsActive()) {
            HandleDriveControls();
            HandleNonChassisMovements();
            TelemetryPrintStatements();

        }

    }


    public void goToTransfer() {
        intakeElbowR.setPosition(R_ELBOW_TRANSFER);
        intakeElbowL.setPosition(L_ELBOW_TRANSFER);
        closeLooselyIntakeClaw();
        sleep(500);
        intakeMotor.setTargetPosition(-MOTOR_TRANSFER_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_UP_POWER);

        intakeState = IntakeState.TRANSFER;
    }

    public void goToPositioning() {
        intakeMotor.setTargetPosition(-MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_DOWN_POWER);
        openOuttakeClaw();
        sleep(500);
        intakeElbowR.setPosition(R_ELBOW_HIGH_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_HIGH_POSITIONING);
        sleep(200);
        intakeElbowR.setPosition(R_ELBOW_LOW_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_LOW_POSITIONING);

        intakeState = IntakeState.POSITIONING;
    }

    public void goToIntakeFromPositioning() {
        intakeElbowR.setPosition(R_ELBOW_INTAKE);
        intakeElbowL.setPosition(L_ELBOW_INTAKE);
        sleep(1000);

        intakeState = IntakeState.INTAKE;
    }

    public void goToPositioningFromIntake() {
        intakeElbowR.setPosition(R_ELBOW_LOW_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_LOW_POSITIONING);

        intakeState = IntakeState.POSITIONING;
    }

    public void goToFullyBack() {
        intakeElbowR.setPosition(R_ELBOW_FULLY_BACK);
        intakeElbowL.setPosition(L_ELBOW_FULLY_BACK);
        sleep(500);
        intakeMotor.setTargetPosition(MOTOR_FULLY_BACK_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_UP_POWER);
    }

    public void intakeSpecimen() {
        intakeMotor.setTargetPosition(-MOTOR_SPECIMEN_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_DOWN_POWER);
        sleep(700);

        intakeElbowR.setPosition(R_ELBOW_SPECIMEN_INTAKE);
        intakeElbowL.setPosition(L_ELBOW_SPECIMEN_INTAKE);

        intakeState = IntakeState.INTAKE;

    }

    public void scoreSpecimen() {
        intakeElbowR.setPosition(R_ELBOW_FULLY_BACK);
        intakeElbowL.setPosition(L_ELBOW_FULLY_BACK);
        sleep(500);
        intakeMotor.setTargetPosition(MOTOR_FULLY_BACK_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_UP_POWER);
        OuttakeShoulder.setPosition(ShoulderPositionRest);
        sleep(100);
        intakeElbowR.setPosition(R_ELBOW_INTAKE);
        intakeElbowL.setPosition(L_ELBOW_INTAKE);

        intakeState = IntakeState.INTAKE;

    }

    public void openIntakeClaw() {
        claw.setPosition(INTAKE_CLAW_OPEN_POSITION);
        clawState = ClawState.OPEN;
    }

    public void closeIntakeClaw() {
        claw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
        clawState = ClawState.CLOSED;
    }

    public void closeLooselyIntakeClaw() {
        claw.setPosition(INTAKE_CLAW_LOOSELY_CLOSED_POSITION);
        clawState = ClawState.CLOSED;
    }

    public void toggleIntakeClaw() {
        if (clawState == ClawState.CLOSED) {
            openIntakeClaw();
            sleep(200);
        } else {
            closeIntakeClaw();
            sleep(200);
            goToPositioningFromIntake();
        }
    }


    private void linearSlidesUp() {
        int currentLeftPosition = linearSlideL.getCurrentPosition();
        int currentRightPosition = linearSlideR.getCurrentPosition();

        if (currentLeftPosition > -LINEAR_SLIDES_MAX_POSITION && currentRightPosition < LINEAR_SLIDES_MAX_POSITION) {
            linearSlideL.setPower(-linearSlidesPower);
            linearSlideR.setPower(linearSlidesPower);
        } else {
            linearSlidesStop();
        }
    }

    private void linearSlidesDown() {
            linearSlideL.setPower(linearSlidesPower);
            linearSlideR.setPower(-linearSlidesPower);
    }

    private void linearSlidesStop() {
        linearSlideL.setPower(0);
        linearSlideR.setPower(0);
    }

    public void ShoulderSpecimen() {
        openOuttakeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionSpecimen);
        outtakeShoulderState = OuttakeShoulderState.SPECIMEN;
    }

    public void ToggleShoulder() {
        if (outtakeShoulderState == OuttakeShoulderState.TRANSFER) {
            ShoulderBasket();
        } else {
            ShoulderTransfer();
        }
    }

    public void ShoulderBasket() {
        closeOuttakeClaw();
        OuttakeShoulder.setPosition(ShoulderPositionBasket);
        outtakeShoulderState = OuttakeShoulderState.BASKET;
    }

    public void ShoulderTransfer() {
        OuttakeShoulder.setPosition(ShoulderPositionTransfer);
        outtakeShoulderState = OuttakeShoulderState.TRANSFER;
    }

    public void ShoulderRest() {
        closeOuttakeClaw();
        sleep(200);
        OuttakeShoulder.setPosition(ShoulderPositionRest);
    }

    public void openOuttakeClaw() {
        OuttakeClaw.setPosition(OUTTAKE_CLAW_DEFAULT_OPEN_POSITION);
        clawState = ClawState.OPEN;
    }

    public void closeOuttakeClaw() {
        OuttakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
        clawState = ClawState.CLOSED;
    }

    public void toggleOuttakeClaw() {
        if (clawState == ClawState.CLOSED) {
            openOuttakeClaw();
            sleep(500);
        } else {
            closeOuttakeClaw();
            sleep(500);
        }
    }

    public void intakeWristStraight() {
        intakeWrist.setPosition(INTAKE_WRIST_STRAIGHT);
        intakeWristState = IntakeWristState.STRAIGHT;
    }

    public void intakeWrist45CC() {
        intakeWrist.setPosition(INTAKE_WRIST_45_CC);
        intakeWristState = IntakeWristState.DIAGONALCC;
    }

    public void intakeWrist90CC() {
        intakeWrist.setPosition(INTAKE_WRIST_90_CC);
        intakeWristState = IntakeWristState.HORIZONTALCC;
    }

    public void intakeWrist45C() {
        intakeWrist.setPosition(INTAKE_WRIST_45_C);
        intakeWristState = IntakeWristState.DIAGONALC;
    }

    public void intakeWrist90C() {
        intakeWrist.setPosition(INTAKE_WRIST_90_C);
        intakeWristState = IntakeWristState.HORIZONTALC;
    }

    public void toggleIntakeWristC() {
        if (intakeWristState == IntakeWristState.STRAIGHT) {
            intakeWrist45C();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.DIAGONALC) {
            intakeWrist90C();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.HORIZONTALC) {
            intakeWristStraight();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.HORIZONTALCC) {
            intakeWrist45CC();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.DIAGONALCC) {
            intakeWristStraight();
            sleep(100);

        }
    }

    public void toggleIntakeWristCC() {
        if (intakeWristState == IntakeWristState.STRAIGHT) {
            intakeWrist45CC();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.DIAGONALCC) {
            intakeWrist90CC();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.HORIZONTALCC) {
            intakeWristStraight();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.HORIZONTALC) {
            intakeWrist45C();
            sleep(100);

        } else if (intakeWristState == IntakeWristState.DIAGONALC) {
            intakeWristStraight();
            sleep(100);

        }
    }

    public void transferSample() {
        closeOuttakeClaw();
        sleep(200);
        openIntakeClaw();

    }

    public static void sleep(int timeInMS) {
        long beginTimeInMS = System.currentTimeMillis();
        long currentTimeInMS;
        do {
            currentTimeInMS = System.currentTimeMillis();
        } while (currentTimeInMS-beginTimeInMS < timeInMS);
    }

    public static void main(String[] str){
        BasicTeleOp.sleep(50000);
    }

        public void hwMapAndEncodersAndDriveDirections () {
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

            intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            linearSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            linearSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            linearSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            linearSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.REVERSE);
        }

        public void InitializedPosition () {
            openIntakeClaw();
            ShoulderTransfer();
            openOuttakeClaw();
        }

        public void HandleDriveControls () {
            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            leftFront.setPower((drive + turn + strafe) * speed);
            rightFront.setPower((drive - turn - strafe) * speed);
            rightRear.setPower((drive - turn + strafe) * speed);
            leftRear.setPower((drive + turn - strafe) * speed);

            if (gamepad1.right_stick_y == 0 || gamepad1.left_stick_x == 0) {
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Update speed based on robot state
            if (intakeState == IntakeState.POSITIONING) {
                speed = POSITIONING_SPEED;

            } else {
                // Handle other speed controls
                if (gamepad1.a) {
                    speed = 0.25;

                } else if (gamepad1.b) {
                    speed = 0.5;

                } else if (gamepad1.y) {
                    speed = 0.75;

                } else if (gamepad1.x) {
                    speed = 1;

                }
            }
        }
        public void HandleNonChassisMovements () {

            if (gamepad2.right_trigger > linearSlidesBufferZone) {
                linearSlidesDown();

            } else if (gamepad2.left_trigger > linearSlidesBufferZone) {
                linearSlidesUp();

            } else {
                linearSlidesStop();
            }

            if (gamepad2.start) {
                if (intakeState == IntakeState.INTAKE) {
                    toggleIntakeClaw();
                } else {
                    toggleOuttakeClaw();
                }
            }

            if (gamepad2.dpad_left) {
                toggleIntakeWristCC();
            }

            if (gamepad2.dpad_right) {
                toggleIntakeWristC();
            }

            if (gamepad2.dpad_up) {
                intakeWristStraight();
            }

            if (gamepad2.dpad_down) {
                goToFullyBack();
            }

            if (gamepad2.b) {
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
                goToPositioning();
                openIntakeClaw();

                HandleDriveControls();
            }
            if (gamepad2.y) {
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
                openIntakeClaw();
                sleep(50);
                goToIntakeFromPositioning();

                HandleDriveControls();
            }
            if (gamepad2.x) {
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
                goToPositioningFromIntake();

                HandleDriveControls();
            }
            if (gamepad2.a) {
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
                closeIntakeClaw();
                sleep(100);
                intakeWristStraight();
                goToTransfer();

                HandleDriveControls();
            }
            if (gamepad2.back) {
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
                transferSample();

                HandleDriveControls();
            }
            if (gamepad2.left_bumper) {
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
                ShoulderBasket();

                HandleDriveControls();

            } else if (gamepad2.right_bumper) {
                rightFront.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                leftRear.setPower(0);
                ShoulderTransfer();

                HandleDriveControls();
            }
        }

        public void TelemetryPrintStatements () {
            telemetry.addData("Speed", speed);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Robot State", intakeState.toString());

            telemetry.addData("Right Slide Position:", linearSlideR.getCurrentPosition());
            telemetry.addData("Left Slide Position:", linearSlideL.getCurrentPosition());
            telemetry.update();
        }
    }