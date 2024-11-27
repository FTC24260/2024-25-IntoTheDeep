package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

@TeleOp(name = "Test Mecanum Field Centric", group = "Tests")
public class TestMecanumFieldCentric extends LinearOpMode {
    private Mecanum mecanum;

    @Override
    public void runOpMode(){
        mecanum = new Mecanum();
        mecanum.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x/2;
            double y = -gamepad1.left_stick_y/2;  // Negated to match standard coordinate system
            double turn = gamepad1.right_stick_x/2;

            mecanum.driveFieldCentric(x, y, turn);

            // Add more detailed telemetry
            telemetry.addData("Inputs", "X: %.2f, Y: %.2f, Turn: %.2f", x, y, turn);
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(mecanum.drivetrain.getRawExternalHeading()));
            telemetry.update();
        }
    }
}