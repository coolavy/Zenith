package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final DoubleArraySubscriber targetSub;
    private double[] data = new double[]{0, 0, 0, 0};

    public Vision() {
        var table = NetworkTableInstance.getDefault().getTable("ZenithVision");
        targetSub = table.getDoubleArrayTopic("target").subscribe(new double[]{0, 0, 0, 0});
    }

    @Override
    public void periodic() {
        data = targetSub.get();
    }

    public boolean hasTarget() { return data[3] > 0; }
    public double getTargetX() { return data[0]; }
    public double getTargetY() { return data[1]; }
    public double getTargetYaw() { return data[2]; }
    public double getClusterSize() { return data[3]; }
}
