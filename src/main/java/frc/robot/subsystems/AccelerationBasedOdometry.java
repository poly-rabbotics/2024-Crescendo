package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AccelerationBasedOdometry {
    private double displacementX = 0.0;
    private double displacementY = 0.0;

    private double velocityX = 0.0;
    private double velocityY = 0.0;

    private double accelerationX = 0.0;
    private double accelerationY = 0.0;

    private Angle orientation = new Angle();

    private long recordedTime;

    /**
     * Creates a new acceleration based odometry instance, assumes starting velocity and 
     * acceleration to be zero.
     * 
     * The given starting position must have a matching unit to the accelerations you give later,
     * e.g. meters and meters per second per second.
     */
    public AccelerationBasedOdometry(Pose2d startingPosition) {
        displacementX = startingPosition.getX();
        displacementY = startingPosition.getY();

        recordedTime = System.currentTimeMillis();
    }

    public Pose2d getPosition() {
        return new Pose2d(displacementX, displacementY, new Rotation2d(orientation.radians()));
    }

    public void update(double accelerationX, double accelerationY, Angle orientation) {
        this.orientation = orientation;
        
        double componentX = 
            accelerationX * Math.cos(orientation.radians())
            + accelerationY * Math.sin(orientation.radians()); 

        double componentY = 
            accelerationX * Math.sin(orientation.radians())
            + accelerationY * Math.cos(orientation.radians()); 

        accelerationX = componentX;
        accelerationY = componentY;

        long currentTime = System.currentTimeMillis();
        long timeElapsed = currentTime - recordedTime;
        
        double deltaTime = 1000.0 * (double)timeElapsed;

        double velocityX = integrate(this.accelerationX, accelerationX, deltaTime, this.velocityX);
        double velocityY = integrate(this.accelerationY, accelerationY, deltaTime, this.velocityY);

        displacementX = integrate(this.velocityX, velocityX, deltaTime, displacementX);
        displacementY = integrate(this.velocityY, velocityY, deltaTime, displacementY);

        this.velocityX = velocityX;
        this.velocityY = velocityY;

        this.accelerationX = accelerationX;
        this.accelerationY = accelerationY;

        recordedTime = currentTime;
    }

    private double integrate(double a, double b, double dx, double c) {
        double rectanglePart = a * dx;
        double delta = b - a;
        double trianglePart = delta * dx * 0.5;
        return rectanglePart + trianglePart + c;
    }
}
