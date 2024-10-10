package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Mecanum extends SubsystemParent
{
    public SampleMecanumDrive drivetrain;

    @Override
    public void init(HardwareMap hwMap) {
        drivetrain = new SampleMecanumDrive(hwMap);
    }

    public void driveFieldCentric(double x, double y, double turn) {
        drivetrain.driveFieldCentric(x, y, turn);
    }

    public void driveRobotCentric(double x, double y, double turn) {
        drivetrain.setWeightedDrivePower(new Pose2d(x, y, turn));
    }
}
