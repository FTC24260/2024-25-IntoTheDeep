package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OuttakeShoulderAndClaw extends LinearOpMode {
    //private Servo OuttakeClaw;
    private Servo OuttakeShoulder;
    @Override
    public void runOpMode() throws InterruptedException {
        //OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeShoulder = hardwareMap.get(Servo.class, "OuttakeShoulder");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.right_bumper){
                OuttakeShoulder.setPosition(1);
                //OuttakeClaw.setPosition(0.4);
            }

            if (gamepad2.left_bumper){
                OuttakeShoulder.setPosition(0);
                //OuttakeClaw.setPosition(1);
            }
        }
    }
}