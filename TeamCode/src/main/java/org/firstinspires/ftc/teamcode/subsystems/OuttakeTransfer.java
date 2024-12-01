package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeTransfer extends SubsystemParent {

    private DcMotorEx OuttakeMotor;
    private Servo OuttakeElbow;

    private final double OUTTAKE_POWER = 0.33;
    private final int MOTOR_OUTTAKE_POSITION = 1000;
    private final int MOTOR_TOILET_POSITION = -1000;

    private final double ELBOW_TOILET = 1;
    private final double ELBOW_SCORE = 0;


    @Override
    public void init(HardwareMap hwMap) {
        OuttakeMotor = hwMap.get(DcMotorEx.class, "OuttakeMotor");
        OuttakeElbow = hwMap.get(Servo.class,"OuttakeElbow");
        OuttakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        OuttakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToBasket(){
        OuttakeMotor.setTargetPosition(MOTOR_OUTTAKE_POSITION);
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeMotor.setPower(OUTTAKE_POWER);
        OuttakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToToilet(){
        OuttakeMotor.setTargetPosition(MOTOR_TOILET_POSITION);
        OuttakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        OuttakeMotor.setPower(OUTTAKE_POWER);
        OuttakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }




}
