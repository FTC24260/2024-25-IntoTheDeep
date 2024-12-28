package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

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
    private Servo OuttakeShoulder;
    private Servo OuttakeClaw;

    private final double INTAKE_POWER = 0.6;
    private final double ShoulderPositionTransfer = 0.35;
    private final double ShoulderPositionSpecimen = 0;
    private final double ShoulderPositionRest = 0.15;
    private final double ShoulderPositionBasket = 0.8;
    private final double linearSlidesPower = 0.1;
    private final double OUTTAKE_CLAW_DEFAULT_OPEN_POSITION = 0.1;
    private final double OUTTAKE_CLAW_CLOSED_POSITION = 0.4;
    private final double linearSlidesBufferZone = 0.1;
    private final int MOTOR_INTAKE_POSITION = 1000;
    private final int MOTOR_TRANSFER_POSITION = 625;
    private final int MOTOR_FULLY_BACK_POSITION = 0;
    private final double L_ELBOW_INTAKE = 0.82;
    private final double R_ELBOW_INTAKE = 0.48;
    private final double L_ELBOW_POSITIONING = 0.73;
    private final double R_ELBOW_POSITIONING = 0.6;
    private final double L_ELBOW_TRANSFER = 0.25;
    private final double R_ELBOW_TRANSFER = 0.95;
    private final double R_ELBOW_FULLY_BACK = 1;
    private final double L_ELBOW_FULLY_BACK = 0.2;
    private final double INTAKE_CLAW_OPEN_POSITION = 0;
    private final double INTAKE_CLAW_LOOSELY_CLOSED_POSITION = 0.31;
    private final double INTAKE_CLAW_CLOSED_POSITION = 0.33;
    private final int R_linearSlidesMax = 1000;
    private final int L_linearSlidesMin = 0;

    private enum ClawState {
        OPEN, CLOSED
    }

    private ClawState clawState = ClawState.CLOSED;

    private enum IntakeState {
        POSITIONING, INTAKE, TRANSFER
    }

    private IntakeState intakeState = IntakeState.TRANSFER;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        double speed = 0.5;

        waitForStart();
        ShoulderRest();
        runtime.reset();

        while (opModeIsActive()) {

            if (gamepad2.right_stick_y > linearSlidesBufferZone) {
                linearSlidesUp();

            } else if (gamepad2.right_stick_y < -linearSlidesBufferZone) {
                linearSlidesDown();

            } else {
                linearSlidesStop();

            } if (gamepad2.start) {
                toggleOuttakeClaw();
            }
            if (gamepad2.b) {
                goToPositioning();
                sleep(500);
                openIntakeClaw();
                speed = 0.1;

            } if (gamepad2.y) {
                goToIntakeFromPositioning();
                closeIntakeClaw();
                closeOuttakeClaw();
                ShoulderTransfer();
                openOuttakeClaw();

            } if (gamepad2.x) {
                goToPositioningFromIntake();
                openIntakeClaw();

            } if (gamepad2.a) {
                goToTransfer();
                openOuttakeClaw();
                sleep(700);
                closeLooselyIntakeClaw();


            } if (gamepad2.back) {
                transferSample();
                sleep(1000);
                goToFullyBack();
            }
            if (gamepad2.left_bumper) {
                ShoulderBasket();

            } else if (gamepad2.right_bumper) {
                ShoulderTransfer();
            }



            // Drive controls
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

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

            // Speed control
            if (gamepad1.a) {
                speed = 0.25;
            } else if (gamepad1.b) {
                speed = 0.5;
            } else if (gamepad1.y) {
                speed = 0.75;
            } else if (gamepad1.x) {
                speed = 1;
            }

            telemetry.addData("Speed", speed);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void goToTransfer() {
        intakeElbowR.setPosition(R_ELBOW_TRANSFER);
        intakeElbowL.setPosition(L_ELBOW_TRANSFER);
        sleep(800);
        intakeMotor.setTargetPosition(MOTOR_TRANSFER_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_POWER);

    }

    public void goToPositioning() {
        intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_POWER);
        sleep(1000);
        intakeElbowR.setPosition(R_ELBOW_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_POSITIONING);
        intakeState = IntakeState.POSITIONING;
    }

    public void goToIntakeFromPositioning() {
        intakeElbowR.setPosition(R_ELBOW_INTAKE);
        intakeElbowL.setPosition(L_ELBOW_INTAKE);
        sleep(1000);
        closeIntakeClaw();
        intakeState = IntakeState.INTAKE;
    }

    public void goToPositioningFromIntake() {
        intakeElbowR.setPosition(R_ELBOW_POSITIONING);
        intakeElbowL.setPosition(L_ELBOW_POSITIONING);

        intakeState = IntakeState.POSITIONING;
    }

    public void goToFullyBack() {
        intakeElbowR.setPosition(R_ELBOW_FULLY_BACK);
        intakeElbowL.setPosition(L_ELBOW_FULLY_BACK);
        intakeMotor.setTargetPosition(MOTOR_FULLY_BACK_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void toggleIntake() {
        if (intakeState == IntakeState.POSITIONING) {
            intakeElbowR.setPosition(R_ELBOW_INTAKE);
            intakeElbowL.setPosition(L_ELBOW_INTAKE);
            intakeState = IntakeState.INTAKE;
        } else if (intakeState == IntakeState.INTAKE) {
            intakeElbowR.setPosition(R_ELBOW_POSITIONING);
            intakeElbowL.setPosition(L_ELBOW_POSITIONING);
            intakeState = IntakeState.POSITIONING;
        }
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
            sleep(500);
        } else {
            closeIntakeClaw();
            sleep(500);
        }
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

    public void transferSample() {
        closeOuttakeClaw();
        sleep(1000);
        openIntakeClaw();

    }
}