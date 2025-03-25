package frc.Lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;

public class AdvancedPose2D extends Pose2d {
    public AdvancedPose2D() {
        super();
    }

    public AdvancedPose2D(Translation2d translation, Rotation2d rotation){
        super(translation, rotation);
    }

    public AdvancedPose2D(double x, double y, Rotation2d rotation){
        super(new Translation2d(x, y), rotation);
    }

    public AdvancedPose2D(double x, double y, double degrees) {
        super(new Translation2d(x, y), Rotation2d.fromDegrees(degrees));
    }

    public AdvancedPose2D(Pose2d pose2d) {
        super(pose2d.getTranslation().getX(), pose2d.getTranslation().getY(), pose2d.getRotation());
    }

    public AdvancedPose2D horizontallyFlip(){
        return new AdvancedPose2D(new Translation2d(FieldConstants.fieldLength - this.getTranslation().getX(),
                                                    this.getTranslation().getY()),
                                  Rotation2d.fromDegrees((this.getRotation().getDegrees() > 0) ?
                                                          180 - this.getRotation().getDegrees() :
                                                        -(180 + this.getRotation().getDegrees())));
    }

    public AdvancedPose2D verticallyFlip() {
        return new AdvancedPose2D(new Translation2d(this.getTranslation().getX(),
                                                    FieldConstants.fieldWidth - this.getTranslation().getY()), 
                                                    Rotation2d.fromDegrees(-this.getRotation().getDegrees()));
    }

    public AdvancedPose2D flipBoth() {
        return new AdvancedPose2D(new Translation2d(FieldConstants.fieldLength - this.getTranslation().getX(),
                                                    FieldConstants.fieldWidth - this.getTranslation().getY()),
                                  Rotation2d.fromDegrees((this.getRotation().getDegrees() > 0) ?
                                                          this.getRotation().getDegrees() - 180 :
                                                          this.getRotation().getDegrees() + 180));
    }

    /**
     * Apply a rotated transfromation to the {@link AdvancedPose2D} object
     * 
     * @param direction the direction of the translation
     * @param translation Y positive goes front and X positive goes Right
     * @param desiredHeading the heading of processed pose
     * @return the transformed {@link AdvancedPose2D} object
     */
    public AdvancedPose2D withVector(Rotation2d direction, Translation2d translation, Rotation2d desiredHeading) {
        double x = translation.getY() * direction.getCos() + translation.getX() * direction.getSin() + this.getX();
        double y = translation.getY() * direction.getSin() - translation.getX() * direction.getCos() + this.getY();
        return new AdvancedPose2D(x, y, desiredHeading);
    }

    /**
     * apply a robot-relative transformation to the {@link AdvancedPose2D} object
     * @param transformation Y positive goes front and X positive goes Right
     * @return the transformed {@link AdvancedPose2D} object
     */
    public AdvancedPose2D withRobotRelativeTransformation(Translation2d transformation) {
        return this.withVector(this.getRotation(), transformation, this.getRotation());
    }

    public AdvancedPose2D withReefAlignment(Alignment alignment, boolean isL4) {
        double backset = alignment == Alignment.center ? AutoAimConstants.algaePosBackset : -AutoAimConstants.coralPosBackset;
        if (alignment == Alignment.blank) backset = 0;
        return new AdvancedPose2D(this.withRobotRelativeTransformation(new Translation2d(AutoAimConstants.offsetFromAlignment.get(alignment),
                                                                       (isL4 ? -.003 : backset))).getTranslation(),
                                                                       Rotation2d.fromDegrees(this.getRotation().getDegrees() +
                                                                       (alignment == Alignment.center ? (this.getRotation().getDegrees() <= -180 ? 180 : -180) : 0)));
    }

    public double getDistance(AdvancedPose2D other) {
        return this.getTranslation().getDistance(other.getTranslation());
    }
}