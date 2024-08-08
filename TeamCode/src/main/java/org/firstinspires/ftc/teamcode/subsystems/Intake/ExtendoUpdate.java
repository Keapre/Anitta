package org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;

public class ExtendoUpdate extends Thread {
    public Robot3 robot;

    public ExtendoUpdate(Robot3 robot) {
        this.robot = robot;
    }

    public void run() {
        robot.extendo.update();
        try {
            Thread.sleep(6);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
