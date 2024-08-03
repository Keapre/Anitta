package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import org.firstinspires.ftc.teamcode.Robot2;

public class SlidesUpdate extends Thread {
    public Robot2 robot;

    public SlidesUpdate(Robot2 robot) {
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
