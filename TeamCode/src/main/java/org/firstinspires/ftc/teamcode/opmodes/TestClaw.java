package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;

@TeleOp
public class TestClaw extends LinearOpMode {
    private IntakeClaw claw = new IntakeClaw();
    private boolean wasAPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);

        telemetry.addLine("Claw Test OpMode Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !wasAPressed) {
                claw.toggleClaw();
                telemetry.addLine("Claw Toggled!");
                telemetry.update();
                wasAPressed = true;
            }

            if (!gamepad1.a) {
                wasAPressed = false;
            }

            sleep(50);
        }
    }
}