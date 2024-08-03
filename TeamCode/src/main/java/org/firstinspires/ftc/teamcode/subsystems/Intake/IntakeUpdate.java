package org.firstinspires.ftc.teamcode.subsystems.Intake;

import org.firstinspires.ftc.teamcode.Robot2;

public class IntakeUpdate extends Thread {

    public Robot2 robot;

    public IntakeUpdate(Robot2 robot) {
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
