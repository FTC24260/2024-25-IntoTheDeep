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
    private Servo intakeElbowR;
    private Servo intakeElbowL;
    private Servo claw;
    private final double INTAKE_POWER = 0.5;
    private final int MOTOR_INTAKE_POSITION = 1584;
    private final int MOTOR_TOILET_POSITION = 774;
    private final double ELBOW_UP = 0.5;
    private final double ELBOW_DOWN = -0.5;
    private final double CLAW_OPEN_POSITION = 0.2;
    private final double CLAW_CLOSED_POSITION = 0.85;
    private final double intakeMotorTicksPerRev = 3895.9;
    private final double intakeMotorCircum = 50.26;
    private final double intakeMotorTicksPerInch = 77.5;
    private final double ticksPerAngle = 8.6;
    enum ClawState {
        OPEN, CLOSED
    }
    private Intake.ClawState clawState = Intake.ClawState.CLOSED;


    @Override
    public void runOpMode() {
        telemetry.addData("niggas in", "paris");
        telemetry.update();


        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeElbowR = hardwareMap.get(Servo.class, "intakeElbowR");
        intakeElbowL = hardwareMap.get(Servo.class, "intakeElbowL");
        claw = hardwareMap.get(Servo.class, "claw");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPosition(CLAW_CLOSED_POSITION);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        //ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //ClawAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        double speed = 0.5;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        openClaw();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.a) {
                goToToilet();
            }
            if (gamepad2.b){
                goToIntake();
            }
            if (gamepad2.x){
                toggleClaw();
            }

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to c
            // combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;
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
    public void goToToilet() {
        openClaw();

        intakeMotor.setTargetPosition(MOTOR_TOILET_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_POWER);
        sleep(5000);
        intakeElbowR.setPosition(ELBOW_DOWN);
        intakeElbowL.setPosition(ELBOW_UP);

    }

    public void goToIntake() {
        intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_POWER);
        sleep(2000);

        intakeElbowR.setPosition(ELBOW_UP);
        intakeElbowL.setPosition(ELBOW_DOWN);
        sleep(1000);
        closeClaw();
    }

    public void openClaw() {
        claw.setPosition(CLAW_OPEN_POSITION);
        clawState = Intake.ClawState.OPEN;
    }

    public void closeClaw() {
        claw.setPosition(CLAW_CLOSED_POSITION);
        clawState = Intake.ClawState.CLOSED;
    }

    public void toggleClaw() {
        if (clawState == Intake.ClawState.CLOSED) {
            openClaw();
            sleep(500);
        } else {
            closeClaw();
            sleep(500);
        }
    }
}
