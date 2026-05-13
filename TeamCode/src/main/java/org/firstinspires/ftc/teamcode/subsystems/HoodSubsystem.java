package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class HoodSubsystem {
    private final Servo hood;
    private double targetPos = .2;

    public HoodSubsystem(Servo hood) {
        this.hood = hood;
        hood.setPosition(targetPos);
    }

    public void setTargetPos(double pos) {
        targetPos = MathUtil.clip(pos, 0.0, 1.0);
    }

    public double getTargetPos() { return targetPos; }

    public void update() {
        hood.setPosition(targetPos);
    }
}
