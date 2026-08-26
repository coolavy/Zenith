// Goes in commands.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class ZenithPursuit extends Command {
    private final Drive drive;
    private final Vision vision;

    private final PIDController xPID = new PIDController(2.5, 0, 0);
    private final PIDController yPID = new PIDController(2.5, 0, 0);
    private final PIDController turnPID = new PIDController(3.5, 0, 0.1);

    public ZenithPursuit(Drive drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            // SEARCH: Spin to find ball
            drive.runVelocity(new ChassisSpeeds(0, 0, 1.5));
            return;
        }

        // CHASE: Drive to X/Y coordinates
        double vx = xPID.calculate(vision.getTargetX(), 0.4); // Stop 0.4m away
        double vy = yPID.calculate(vision.getTargetY(), 0);
        double omega = turnPID.calculate(Math.toRadians(vision.getTargetYaw()), 0);

        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public void end(boolean interrupted) {
        drive.runVelocity(new ChassisSpeeds());
    }
}
