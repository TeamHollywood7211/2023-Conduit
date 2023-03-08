package frc.robot;

public final class Constants {
//========DRIVETRAIN CONSTANTS=======================================================================================

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6604;//26.0
        //The left-to-right distance between the drivetrain wheels should be measured from center to center.
        //The front-to-back distance between the drivetrain wheels should be measured from center to center.

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.8128;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(177.626); // 85.25 for practice bot 177.626 for new

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(10.458); // 25.92 for practice bot 11.953 for new

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 21; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(254); // 57.21 for practice bot 252.333 for new

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 31;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 32;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 33;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(275.097); // 159.61 for practice bot -87.515 for new

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 35;
    public static final int STEER_MOTOR_CURRENT_LIMIT = 20;
    public static final int NOMINAL_DRIVE_VOLTAGE = 12;

    public static double AUTON_TRANSLATE_P = 1;
    public static double AUTON_ROTATE_P = 0.56;
    
    // public static HashMap<String, Command> eventMap = new HashMap<>();;

//====ARM CONSTANTS================================================================================================

    public static final double armkP = 0.03;
    public static final double armkI = 0;
    public static final double armkD = 0;

    public static final double armHighTarget = 99.95;
    public static final double armMidTarget = 85.98;
    public static final double armLowTarget = 33;
    public static final double armStoredTarget = 0;

    public static final int ARM_MOTOR_ID = 41;

    public static final int ARM_MOTOR_CURRENT_LIMIT = 20;
    public static final int ARM_MOTOR_INIT_CURRENT_LIMIT = 0;

    public static final double ARM_MOTOR_INIT_VELOCITY_MIN = 0.1;

    public static final double MANUAL_ARM_ADJUST_DEADZONE = 0.1;
    public static final double MANUAL_ARM_ADJUST_POWER_MULTIPLIER = 11;

//====GRIP CONSTANTS===============================================================================================
    
    public static final int GRIP_MOTOR_ID = 42;

    public static final double GRIP_MOTOR_SPEED = 0.5;

    public static final int GRIP_MOTOR_INIT_CURRENT_LIMIT = 0;
    public static final int GRIP_MOTOR_CURRENT_LIMIT = 20;

    public static final double GRIP_MOTOR_INIT_VELOCITY_MIN = 0.1;

    public static final double gripkP = 0.1;
    public static final double gripkI = 0;
    public static final double gripkD = 0;

    public static final double gripConeTarget = -148;
    public static final double gripCubeTarget = -89;
    public static final double gripOutTarget = 0;

    public static final int GRIP_WHEELS_MOTOR_CURRENT_LIMIT = 20;

//====COUNTERWEIGHT CONSTANTS======================================================================================

    public static final int COUNTERWEIGHT_MOTOR_ID = 43;

    public static final double counterweightkP = 0.1;
    public static final double counterweightkI = 0;
    public static final double counterweightkD = 0;

    public static final double counterweightHighTarget = -107.587;
    public static final double counterweightMidTarget = -50.893;
    public static final double counterweightLowTarget = 0;

    public static final int COUNTERWEIGHT_CURRENT_LIMIT = 20;
    public static final int COUNTERWEIGHT_INIT_CURRENT_LIMIT = 15;

    public static final double MANUAL_COUNTERWEIGHT_ADJUST_POWER_MULTIPLER = 1;

//====ANTI TIP CONSTANTS===========================================================================================

    public static final double tipkP = 1;
    public static final double tipkI = 0;
    public static final double tipkD = 0;

//====OTHER CONSTANTS==============================================================================================

    public static final int LED_PWM_PORT = 30; //plz change later, not actual value, thx :)
    public static final double gripTriggerDeadzone = 0.2;
}
