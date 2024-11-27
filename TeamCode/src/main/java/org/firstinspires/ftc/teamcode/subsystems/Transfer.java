package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer extends SubsystemParent {

    private DcMotorEx intakeMotor;
    private Servo intakeElbowR;
    private Servo intakeElbowL;

    private final double INTAKE_POWER = 0.33;
    private final int MOTOR_INTAKE_POSITION = 1000;
    private final int MOTOR_TOILET_POSITION = -1000;

    private final double ELBOW_UP = 0.25;
    private final double ELBOW_DOWN = 0;


    @Override
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeElbowR = hwMap.get(Servo.class,"intakeElbowR");
        intakeElbowL = hwMap.get(Servo.class,"intakeElbowL");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void goToIntake(){
        intakeMotor.setTargetPosition(MOTOR_INTAKE_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(INTAKE_POWER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeElbowR.setPosition(ELBOW_UP);
        intakeElbowL.setPosition(ELBOW_UP);
    }

    public void goToToilet(){
        intakeMotor.setTargetPosition(MOTOR_TOILET_POSITION);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(-INTAKE_POWER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeElbowR.setPosition(ELBOW_DOWN);
        intakeElbowL.setPosition(ELBOW_DOWN);
    }




}
