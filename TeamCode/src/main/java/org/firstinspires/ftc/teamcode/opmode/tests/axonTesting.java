package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

public class axonTesting extends LinearOpMode {
    String name = "name";
    String nameEncoder = "nameEncoder";
    Servo servo;
    AnalogInput encoder;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, name);
        encoder = hardwareMap.get(AnalogInput.class, nameEncoder);

        long speedStartTime = 0,speedEndTime = 0;
        double min = 0,max = 1;
        double minAngle =0,maxAngle = 0;
        double positionPerRadian = 0;
        double speed = 0;

        waitForStart();

        servo.setPosition(0);
        servoDone(encoder);
        minAngle = getEncAngle(encoder);

        servo.setPosition(1);
        servoDone(encoder);
        maxAngle = getEncAngle(encoder);

        servo.setPosition(0);
        servoDone(encoder);

        servo.setPosition(1);
        speedStartTime = System.currentTimeMillis();
        servoDone(encoder);
        speedEndTime = System.currentTimeMillis();



        positionPerRadian = (max - min) / (maxAngle - minAngle);
        speed = (maxAngle - minAngle) / (speedEndTime - speedStartTime) / 1000.0;
    }

    public static double getEncAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * 2 * Math.PI;
    }

    public static void servoDone(AnalogInput encoder) {
        double lastAngle = getEncAngle(encoder);

        long startTime = System.currentTimeMillis();

        while(System.currentTimeMillis() - startTime < 350) {
            double currentAngle = getEncAngle(encoder);
            if(Math.abs(currentAngle - lastAngle) > Math.toRadians(3.5)) {
                startTime = System.currentTimeMillis();
            }else {
                break;
            }
            lastAngle = currentAngle;
        }
    }

}
