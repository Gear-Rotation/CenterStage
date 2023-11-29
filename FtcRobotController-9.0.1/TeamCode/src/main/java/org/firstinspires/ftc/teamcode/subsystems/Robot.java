package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public Drivetrain drive;
    public Lift lift;

    public Intake intake;
    public LinearActuator linearActuator;

    private List<Subsystem> subsystems;
    private Telemetry telemetry;
    public Timer timer;

    private LinearOpMode opMode;

    public Robot(LinearOpMode opMode, Telemetry telemetry, Pose2d initialPose) {
        this.telemetry = telemetry;
        this.opMode = opMode;
        timer = new Timer(opMode);

        drive = new Drivetrain(opMode, telemetry, initialPose);
        lift = new Lift(opMode, telemetry);
        intake = new Intake(opMode, telemetry);
        linearActuator = new LinearActuator(opMode, telemetry);

        subsystems = new ArrayList<>();

        subsystems.add(drive);
        subsystems.add(lift);
        subsystems.add(intake);
        subsystems.add(linearActuator);
    }

    public Robot(LinearOpMode opMode, Telemetry telemetry) {
        this(opMode, telemetry, new Pose2d(0,0,0));
    }

    public void loop() {
        for (Subsystem s : subsystems) {
            s.update();
        }

        telemetry.update();
    }

}