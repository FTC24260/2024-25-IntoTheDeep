package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class TestHub extends LinearOpMode {
    public DcMotor linearSlides;
    public DcMotor ArmMotor;
    public DcMotor ClawAngle;

    public final double SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        //linearSlides = hardwareMap.get(DcMotorEx.class, "linearSlide");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        ClawAngle = hardwareMap.get(DcMotor.class, "ClawAngle");


        waitForStart();

        while(opModeIsActive()){
            /*if(gamepad1.a){
                linearSlides.setPower(SPEED);
            }*/
            if(gamepad1.b){
                ArmMotor.setPower(SPEED);

            }
            if(gamepad1.x){
                ClawAngle.setPower(SPEED);
            }


        }


    }


}
