package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public abstract class AutoOpMode extends LinearOpMode {
    public Robot robot;
    public abstract void setup();
    public abstract void run();

    public Pose2d getInitialPose() {
        return new Pose2d(0,0,0);
    }

    public Pose2d getCurrentPose() {
        return robot.drive.roadRunnerDrive.getPoseEstimate();
    }
}
