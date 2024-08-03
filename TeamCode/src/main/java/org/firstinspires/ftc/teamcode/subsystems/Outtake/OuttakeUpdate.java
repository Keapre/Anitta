package org.firstinspires.ftc.teamcode.subsystems.Outtake;

import org.firstinspires.ftc.teamcode.Robot2;

public class OuttakeUpdate extends Thread {

    public Robot2 robot;

    public OuttakeUpdate(Robot2 robot) {
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
