// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Swerve.modules.Amalia.TopMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Swerve {
        public static class modules {
            public class Idan {
                public class TopMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class ButtomMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class AngleController {
                    public static final double kp = 10;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class VelocityController {
                    public static final double kp = 0.5;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }
            }

            public class Amalia {
                public class TopMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class ButtomMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class AngleController {
                    public static final double kp = 10;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class VelocityController {
                    public static final double kp = 0.5;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }
            }

            public class Iris {
                public class TopMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class ButtomMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class AngleController {
                    public static final double kp = 10;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class VelocityController {
                    public static final double kp = 0.5;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }
            }

            public class Galit {
                public class TopMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class ButtomMotor {
                    public static final double kp = 0;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class AngleController {
                    public static final double kp = 10;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }

                public class VelocityController {
                    public static final double kp = 0.5;
                    public static final double ki = 0;
                    public static final double kff = 0;
                    public static final double kd = 0;
                }
            }

            public static double[][] get_contstants_by_name(String name) throws RuntimeException {
                double[][] return_arr = new double[4][4];
                if (name == "Galit") {
                    return_arr[0][0] = Galit.TopMotor.kd;
                    return_arr[0][1] = Galit.TopMotor.ki;
                    return_arr[0][2] = Galit.TopMotor.kp;
                    return_arr[0][3] = Galit.TopMotor.kff;
                    return_arr[1][0] = Galit.ButtomMotor.kd;
                    return_arr[1][1] = Galit.ButtomMotor.ki;
                    return_arr[1][2] = Galit.ButtomMotor.kp;
                    return_arr[1][3] = Galit.ButtomMotor.kff;
                    return_arr[2][0] = Galit.AngleController.kd;
                    return_arr[2][1] = Galit.AngleController.ki;
                    return_arr[2][2] = Galit.AngleController.kp;
                    return_arr[2][3] = Galit.AngleController.kff;
                    return_arr[3][0] = Galit.VelocityController.kd;
                    return_arr[3][1] = Galit.VelocityController.ki;
                    return_arr[3][2] = Galit.VelocityController.kp;
                    return_arr[3][3] = Galit.VelocityController.kff;
                } else if (name == "Idan") {
                    return_arr[0][0] = Idan.TopMotor.kd;
                    return_arr[0][1] = Idan.TopMotor.ki;
                    return_arr[0][2] = Idan.TopMotor.kp;
                    return_arr[0][3] = Idan.TopMotor.kff;
                    return_arr[1][0] = Idan.ButtomMotor.kd;
                    return_arr[1][1] = Idan.ButtomMotor.ki;
                    return_arr[1][2] = Idan.ButtomMotor.kp;
                    return_arr[1][3] = Idan.ButtomMotor.kff;
                    return_arr[2][0] = Idan.AngleController.kd;
                    return_arr[2][1] = Idan.AngleController.ki;
                    return_arr[2][2] = Idan.AngleController.kp;
                    return_arr[2][3] = Idan.AngleController.kff;
                    return_arr[3][0] = Idan.VelocityController.kd;
                    return_arr[3][1] = Idan.VelocityController.ki;
                    return_arr[3][2] = Idan.VelocityController.kp;
                    return_arr[3][3] = Idan.VelocityController.kff;
                } else if (name == "Amalia") {
                    return_arr[0][0] = Amalia.TopMotor.kd;
                    return_arr[0][1] = Amalia.TopMotor.ki;
                    return_arr[0][2] = Amalia.TopMotor.kp;
                    return_arr[0][3] = Amalia.TopMotor.kff;
                    return_arr[1][0] = Amalia.ButtomMotor.kd;
                    return_arr[1][1] = Amalia.ButtomMotor.ki;
                    return_arr[1][2] = Amalia.ButtomMotor.kp;
                    return_arr[1][3] = Amalia.ButtomMotor.kff;
                    return_arr[2][0] = Amalia.AngleController.kd;
                    return_arr[2][1] = Amalia.AngleController.ki;
                    return_arr[2][2] = Amalia.AngleController.kp;
                    return_arr[2][3] = Amalia.AngleController.kff;
                    return_arr[3][0] = Amalia.VelocityController.kd;
                    return_arr[3][1] = Amalia.VelocityController.ki;
                    return_arr[3][2] = Amalia.VelocityController.kp;
                    return_arr[3][3] = Amalia.VelocityController.kff;
                } else if (name == "Iris") {
                    return_arr[0][0] = Iris.TopMotor.kd;
                    return_arr[0][1] = Iris.TopMotor.ki;
                    return_arr[0][2] = Iris.TopMotor.kp;
                    return_arr[0][3] = Iris.TopMotor.kff;
                    return_arr[1][0] = Iris.ButtomMotor.kd;
                    return_arr[1][1] = Iris.ButtomMotor.ki;
                    return_arr[1][2] = Iris.ButtomMotor.kp;
                    return_arr[1][3] = Iris.ButtomMotor.kff;
                    return_arr[2][0] = Iris.AngleController.kd;
                    return_arr[2][1] = Iris.AngleController.ki;
                    return_arr[2][2] = Iris.AngleController.kp;
                    return_arr[2][3] = Iris.AngleController.kff;
                    return_arr[3][0] = Iris.VelocityController.kd;
                    return_arr[3][1] = Iris.VelocityController.ki;
                    return_arr[3][2] = Iris.VelocityController.kp;
                    return_arr[3][3] = Iris.VelocityController.kff;
                } else {
                    throw new RuntimeException(name + "not in names");
                }
                return return_arr;
            }
        }
    }
}
