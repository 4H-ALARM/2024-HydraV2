package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.configs.moduleConfig;

public class RobotConstants {

    public static final class SwerveConstants {
        public static final moduleConfig mod0 = new moduleConfig(
                12,
                false,
                13,
                false,
                11,
                Rotation2d.fromRotations(-0.271240),
                0
        );

        public static final moduleConfig mod1 = new moduleConfig(
                22,
                false,
                23,
                false,
                21,
                Rotation2d.fromRotations(-0.063477),
                1
        );

        public static final moduleConfig mod2 = new moduleConfig(
                32,
                false,
                33,
                false,
                31,
                Rotation2d.fromRotations(-0.271729),
                2
        );

        public static final moduleConfig mod3 = new moduleConfig(
                42,
                false,
                43,
                false,
                41,
                Rotation2d.fromRotations(-0.94),
                3
        );

    }


}
