package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;

@TeleOp
public class TestClaw extends LinearOpMode {

    private OuttakeClaw claw = new OuttakeClaw();

    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                claw.toggleClaw();
            }
        }
    }
}
