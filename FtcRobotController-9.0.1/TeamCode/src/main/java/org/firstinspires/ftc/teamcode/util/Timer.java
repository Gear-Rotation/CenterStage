package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    private ElapsedTime timer;
    private LinearOpMode opMode;

    public Timer(LinearOpMode opMode) {
        timer = new ElapsedTime();
        this.opMode = opMode;
    }

    /**
     *
     * @param time in milliseconds
     */
    public void wait(int time) {
        resetTimer();
        while (opMode.opModeIsActive() && getTime() < time);
    }

    /**
     *
     * @return time in millseconds
     */
    public double getTime() {
        return timer.milliseconds();
    }

    public void resetTimer() {
        timer.reset();
    }
}