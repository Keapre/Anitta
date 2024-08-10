package org.firstinspires.ftc.teamcode.subsystems.EndGame;

import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides;

public class Hang {
    public enum HangStatus {
        EXTEND,
        RETRACT,
        IDLE
    }

    private double target = 0;
    public HangStatus status = HangStatus.IDLE;
    Slides slides;

    public Hang(Slides slides) {
        this.slides = slides;
    }

    public void setExtend() {
        this.status = HangStatus.EXTEND;
    }

    public void setRetract() {
        this.status = HangStatus.RETRACT;
    }

    public void setIdle() {
        this.status = HangStatus.IDLE;
    }

    public void update() {
        switch (status) {
            case EXTEND:
                slides.sMotor1.setPower(1);
                slides.sMotor2.setPower(1);
                break;
            case RETRACT:
                slides.sMotor1.setPower(-1);
                slides.sMotor2.setPower(-1);
                break;
            case IDLE:
                slides.forceShutDown();
                break;
        }
    }

}
