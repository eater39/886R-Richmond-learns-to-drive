#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-1, 12, -2});    	//pros::MotorGearset::blue
pros::MotorGroup right_mg({-20, 18, 9});  //pros::MotorGearset::blue
pros::Motor stage1(-16);
pros::Motor stage2(-13);

pros::adi::Pneumatics tongue('H', false);
pros::adi::Pneumatics centerupper('A', false);
pros::adi::Pneumatics wing('G', true);
pros::adi::Pneumatics pushdown('E', false);
pros::adi::Pneumatics clamp('F', false);
pros::Rotation rotation_horizontal(11);
pros::Rotation rotation_vertical(-10);
pros::Imu imu(21);
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.5, lemlib::Omniwheel::NEW_4, 343, 2);
lemlib::TrackingWheel horizontal(&rotation_horizontal, lemlib::Omniwheel::NEW_2, -2.5);
lemlib::TrackingWheel vertical(&rotation_vertical, lemlib::Omniwheel::NEW_275, -1);
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP). //means going forward backward.. we dont use I, we use P and D 
                                              0, // integral gain (kI)
                                              13, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.9, // proportional gain (kP) //same thing  
                                              0, // integral gain (kI)
                                              27.8, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::Chassis chassis(
	drivetrain, // drivetrain configuration // odometry sensors
	lateral_controller, // lateral PID controller
	angular_controller,
	sensors // angular PID controller
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
		pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
	static pros::Task screen_task([]() {
		while (true) {
			const auto pose = chassis.getPose();
			pros::lcd::print(0, "X: %f", pose.x); // x position in inches
			pros::lcd::print(1, "Y: %f", pose.y); // y position in inches
			pros::lcd::print(2, "Theta: %f", pose.theta); // heading in degrees
			pros::delay(50);
		}
	});
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
		//chassis.moveToPoint(0, 24, 10000);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
	
		int dir = master.get_analog(ANALOG_LEFT_Y);    
		int turn = master.get_analog(ANALOG_RIGHT_X); 
		left_mg.move(dir + turn);                   
		right_mg.move(dir - turn);      
		
		//block collection
		//intake
		if (master.get_digital(DIGITAL_R1)){
			stage1.move(127);
			stage2.brake();
		}
		else if (master.get_digital(DIGITAL_R2)){
			stage1.move(-127);
			stage2.brake();
		}
		//outtake
		else if (master.get_digital(DIGITAL_L1)){
			stage1.move(127);
			stage2.move(127);
	
		}
		else if (master.get_digital(DIGITAL_L2)){
			stage1.move(100);
			stage2.move(127);
			centerupper.extend();
		}
		else{
			stage1.move(0);
			stage2.move(0);
			centerupper.retract();
		}
		//pneumatics
		if(master.get_digital_new_press(DIGITAL_B))
		{
			tongue.toggle();
		}
		if(master.get_digital(DIGITAL_Y)){
			wing.retract();
		}
		else{
			wing.extend();
		}
		if(master.get_digital_new_press(DIGITAL_RIGHT)){
			clamp.toggle();
		}
		if (master.get_digital_new_press(DIGITAL_DOWN)){
			pushdown.toggle();
		}
			
		pros::delay(20);                              
	}
}