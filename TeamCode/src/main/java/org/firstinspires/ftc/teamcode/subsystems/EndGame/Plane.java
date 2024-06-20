package org.firstinspires.ftc.teamcode.subsystems.EndGame;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityDevice;

public class Plane {
    public enum status {
        IN,
        OUT
    }
    Servo plane;
    public status Status;
    double powerPosition = 0.69; //TODO:change this value

    double[] positions = new double[]{0.10,0.69};
    double close = 0.10;
    public Plane(HardwareMap hardwareMap) {
        this.plane = new CachingServo(hardwareMap.get(Servo.class,"plane"));
        this.plane.setPosition(positions[0]);
        this.Status = status.IN;
    }

    public void initiliaze() {
        this.plane.setPosition(close);
    }

    public void setOut() {
        this.Status = status.OUT;
    }
    void update() {
        switch (Status) {
            case IN:
                this.plane.setPosition(positions[0]);
                break;
            case OUT:
                this.plane.setPosition(positions[1]);
                break;

        }
    }

}
