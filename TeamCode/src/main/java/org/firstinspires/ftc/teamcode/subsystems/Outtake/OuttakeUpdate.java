package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;

public class OuttakeUpdate extends Thread {

    public Robot3 robot;

    public OuttakeUpdate(Robot3 robot) {
        this.robot = robot;
    }

    public void run() {
        robot.outtake.update();
        try {
            Thread.sleep(6);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
