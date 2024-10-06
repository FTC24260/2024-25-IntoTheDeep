package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;


            mecanum.drive(x, y, turn);

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("turn", turn);
            telemetry.update();

        }
    }
}
