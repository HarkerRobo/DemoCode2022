// package frc.robot.util;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import frc.robot.RobotMap;
// import frc.robot.subsystems.Drivetrain;

// public class SwerveControllerLoop {
//   private double T_kP_POS;
//   private double T_kP_VEL;
//   private double R_kP_POS;
//   private double R_kP_VEL;

//   public static double getRotationOnlyOutput(
//       double curRotVelocity, double headingSetpoint, double headingVelSetpoint) {
//     double curHeading =
//         Drivetrain.getInstance()
//             .getPoseEstimator()
//             .getEstimatedPosition()
//             .getRotation()
//             .getDegrees();
//     while (Math.abs(headingSetpoint - curHeading) > 360.0) {
//       headingSetpoint += Math.signum(headingSetpoint - curHeading) * 360.0;
//     }
//     return positionGains[2] * (headingSetpoint - curHeading)
//         + velocityGains[2] * (headingVelSetpoint - curRotVelocity);
//   }

//   public static double getRotationOnlyOutput(double curRotVelocity, double headingSetpoint) {
//     return getRotationOnlyOutput(curRotVelocity, headingSetpoint, 0.0);
//   }

//   public static class SwerveControllerLoopBuilder {
//     private double[] set, velSet, maxError = {}, maxVel = {4.0, Math.PI};
//     private boolean x, y, theta;

//     private static final Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -1
// / RobotMap.ROBOT_LOOP);
//     private static final Matrix<N2, N1> B = VecBuilder.fill(0.0, 1.0 / RobotMap.ROBOT_LOOP);

//     public SwerveControllerLoopBuilder() {
//       set = new double[3];
//       velSet = new double[3];
//       maxError = new double[4];
//     }

//     public SwerveControllerLoopBuilder setTheta(double setpoint) {

//     }
//   }
// }
