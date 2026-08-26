// Goes on RoboRio/Systemcore

public class BallTracker extends Command {
    private final Drive drive;
    private final Vision vision; // Your NT4 reading subsystem
    private final PIDController xPID = new PIDController(2.0, 0, 0);
    private final PIDController yPID = new PIDController(2.0, 0, 0);
    private final PIDController turnPID = new PIDController(3.0, 0, 0.05);

    public BallTracker(Drive drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            drive.runVelocity(new ChassisSpeeds(0, 0, 1.0)); // Search spin
            return;
        }

        double xMeters = vision.getTargetX(); // Forward
        double yMeters = vision.getTargetY(); // Lateral
        double yawDeg  = vision.getTargetYaw();

        // Calculate Speeds
        double vx = xPID.calculate(-xMeters, -0.3); // Stop 0.3m away
        double vy = yPID.calculate(-yMeters, 0);
        double omega = turnPID.calculate(Math.toRadians(yawDeg), 0);

        // Chase the ball
        drive.runVelocity(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && vision.getDistance() < 0.4;
    }
}
