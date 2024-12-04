package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;

@TeleOp
public class TestClaw extends LinearOpMode {
    final private IntakeClaw claw = new IntakeClaw();

    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.toggleClaw();
                telemetry.addData("ToggleClaw", claw.getClawPosition().toString());
            }
            sleep(50);
        }
    }
}