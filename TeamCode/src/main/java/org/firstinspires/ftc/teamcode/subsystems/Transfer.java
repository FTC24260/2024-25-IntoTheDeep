package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer extends SubsystemParent {

    private DcMotorEx intakeMotor;
    private Servo intakeElbowR;
    private Servo intakeElbowL;

    @Override
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeElbowR = hwMap.get(Servo.class,"intakeElbowR");
        intakeElbowL = hwMap.get(Servo.class,"intakeElbowL");
    }



}
