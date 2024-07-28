package org.firstinspires.ftc.teamcode.subsystems.Vision;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Husky{

    private final int READ_PERIOD = 1;
    private final int FRAME_WIDTH = 320;
    private int LEFT_THRESHOLD = FRAME_WIDTH / 3;
    private int RIGHT_THRESHOLD = 2 * FRAME_WIDTH / 3;



    private HuskyLens huskyLens;
    private HuskyLens.Block[] blocks;

    public Husky(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class,"huskyLens");

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        updateBlocks();
    }

    public void updateBlocks() {
        blocks = huskyLens.blocks();
    }

    public int getLocation() {
        updateBlocks();

        int location = 0;
        int x = blocks[0].x;
        int y = blocks[0].y;

        if(x <= LEFT_THRESHOLD) {
            location = 1;
        }else if(x > LEFT_THRESHOLD && x <= RIGHT_THRESHOLD) {
            location = 2;
        }else {
            location = 3;
        }
        return location;
    }
}