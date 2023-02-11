package frc.robot;
public final class Constants {
//========DRIVETRAIN CONSTANTS=======================================================================================

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 24.0;
        //The left-to-right distance between the drivetrain wheels should be measured from center to center.
        //The front-to-back distance between the drivetrain wheels should be measured from center to center.

    public static final double DRIVETRAIN_WHEELBASE_METERS = 26.0;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(85); // 85.25

<<<<<<< Updated upstream
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(26); // 25.92
=======
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11;  
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;  
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(11.953); // 25.92 for practice bot 11.953 for new
>>>>>>> Stashed changes

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 21; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 22; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(57.21); // 57.21

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 31; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 32; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 33; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(160); // 159.61

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 20;
    public static final int STEER_MOTOR_CURRENT_LIMIT = 20;
    public static final int NOMINAL_DRIVE_VOLTAGE = 11;

//====ARM CONSTANTS================================================================================================

    public static final double armkP = 0.1;
    public static final double armkI = 0;
    public static final double armkD = 0;

    public static final double armHighTarget = 100;
    public static final double armMidTarget = 50;
    public static final double armLowTarget = 0;

    public static final double counterweightkP = 0;
    public static final double counterweightkI = 0;
    public static final double counterweightkD = 0;

    public static final double counterweightHighTarget = 100;
    public static final double counterweightMidTarget = 50;
    public static final double counterweightLowTarget = 0;

    public static final int ANGLE_MOTOR_ID = 41; //temp, change to actual ID.
    public static final int GRIP_MOTOR_ID = 42;
    public static final int COUNTERWEIGHT_MOTOR_ID = 43;
    public static final int LEFT_GRIP_WHEELS_MOTOR_ID = 44;
    public static final int RIGHT_GRIP_WHEELS_MOTOR_ID = 45;

    public static final int ANGLE_MOTOR_CURRENT_LIMIT = 20;
    public static final int GRIP_MOTOR_CURRENT_LIMIT = 20;
    public static final int GRIP_WHEELS_MOTOR_CURRENT_LIMIT = 20;

//====OTHER CONSTANTS==============================================================================================

    public static final int LED_PWM_PORT = 30; //plz change later, not actual value, thx :)

}
