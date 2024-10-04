package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Mecanum extends SubsystemParent
{
    public SampleMecanumDrive drivetrain;

    @Override
    public void init(HardwareMap hwMap) {
        drivetrain = new SampleMecanumDrive(hwMap);
    }

    public void drive(double x, double y, double turn) {
        drivetrain.driveFieldCentric(x, y, turn);
    }
}
