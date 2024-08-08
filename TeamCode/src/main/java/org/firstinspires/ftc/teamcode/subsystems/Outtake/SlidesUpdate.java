package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;

public class SlidesUpdate extends Thread {
    public Robot3 robot;

    public SlidesUpdate(Robot3 robot) {
        this.robot = robot;
    }

    public void run() {
        robot.slides.update();
        try {
            Thread.sleep(6);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
