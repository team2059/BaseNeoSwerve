package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Test {

    public static void main(String[] args) {

        double yawLLRadians = 0;
        double xLL = 0;
        double yLL = 0;
        double xCameraOffset = 0;
        double yCameraOffset = 0;

        // robot final to robot initial rotation (used to rotate vectors in rf frame to
        // ri frame)
        Rotation2d rf_to_ri = new Rotation2d(yawLLRadians - Math.PI);

        // april tag in robot final coordiante frame
        Translation2d A_rf = new Translation2d(11, 0);

        // april tag in limelight initial coordinate frame
        Translation2d A_l0 = new Translation2d(xLL, yLL);

        // limelight in robot initial coordinate frame
        Translation2d Originl0_rO = new Translation2d(xCameraOffset, yCameraOffset);

        // april tag in robot initial coordinate frame
        Translation2d A_r0 = A_l0.plus(Originl0_rO);

        Translation2d result = A_r0.minus(A_rf.rotateBy(rf_to_ri));

        System.out.println(result.toString());

    }

}
