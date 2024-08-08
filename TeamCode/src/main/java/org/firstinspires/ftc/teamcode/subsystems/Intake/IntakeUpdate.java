package org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Robot3;

public class IntakeUpdate extends Thread {

    public Robot3 robot;

    public IntakeUpdate(Robot3 robot) {
        this.robot = robot;
    }

    public void run() {
        robot.intake.update();
        try {
            Thread.sleep(6);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
