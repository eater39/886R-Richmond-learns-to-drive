#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-1, 12, -2});    	//pros::MotorGearset::blue, left motorgroup
pros::MotorGroup right_mg({-20, 18, 9});  //pros::MotorGearset::blue, right motorgroup
pros::Motor stage1(-16); // stage 1 of the intake
pros::Motor stage2(-13); // stage 2 of the intake

pros::adi::Pneumatics tongue('H', false); // tongue, also know as the scraper, gets blocks from the loader.
pros::adi::Pneumatics centerupper('A', false);
pros::adi::Pneumatics wing('G', true); // wing de-score


pros::adi::Pneumatics centerupperdescore('F', false);
pros::Rotation rotation_horizontal(11); // horizontal tracking wheeel
pros::Rotation rotation_vertical(-10); // vertical tracking wheel
pros::Imu imu(21); // IMU
lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.5, lemlib::Omniwheel::NEW_4, 343, 2);
lemlib::TrackingWheel horizontal(&rotation_horizontal, lemlib::Omniwheel::NEW_2, 4);
lemlib::TrackingWheel vertical(&rotation_vertical, lemlib::Omniwheel::NEW_275, 0.05);
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(7.3, // proportional gain (kP). //means going forward backward.. we dont use I, we use P and D 
                                              0, // integral gain (kI)
                                              29, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.9, // proportional gain (kP) //same thing  
                                              0, // integral gain (kI)
                                              33.5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
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
void rightsideauto()
{
		    // set position to x:0, y:24, heading:0
			//blueright
    chassis.setPose(9.5, -49, 27.8); //starting position
		stage1.move(100); // turn on intake (stage 1)
		chassis.moveToPoint(24, -20, 6000, {.maxSpeed = 80}, true); // move to the 3 blue blocks
		pros::delay(500);
		chassis.waitUntilDone();
		chassis.moveToPoint(51,-45, 10000, {.forwards = false}, true); // move between long goal and loader
		chassis.waitUntilDone();
		chassis.turnToHeading(180, 500);// turn to face loader
		chassis.waitUntilDone();
		tongue.extend(); // extend scraper 
		pros::delay(250);
		chassis.moveToPoint(51.2, -59.5, 840, {.maxSpeed = 127}, true); //extract blocks (first 3) from loader
		chassis.waitUntilDone();
		//move to long goal
		chassis.moveToPose(52.7, -20, 180, 1700, { .forwards = false, .maxSpeed = 127, }, true);
		//reverse to prevent clogged blocks
		stage1.move(-20);
		chassis.waitUntilDone();
		//score blocks to long goal
		stage1.move(127);
		stage2.move(127);
		pros::delay(1200);
		chassis.moveToPose(46, -40, 180, 1800,{.forwards = true, .maxSpeed = 100}, true); 
		//back away from long goal
		stage1.move(0);
		stage2.move(0);

		chassis.waitUntilDone();
		//arm down
		wing.retract();
		chassis.moveToPoint(51, -13, 1500, {.forwards = false, .maxSpeed = 127}, true);
		//push blocks into goal
		tongue.retract();
		//lock robot
}
/**
 * @brief leftsideauto is a function that will autonomously score blocks into the goals on the left side of the field.
 * It first sets the initial position of the robot, then moves between the long goal and the loader, turns to face the loader, extends the tongue, extracts blocks from the loader, drives to the long goal, scores the blocks, moves away from the long goal, turns to face the blocks, retracts the tongue, moves to the blocks, turns to face the mid goal, scores the mid goal, moves to the long goal, stops the rollers, turns inside the goal, wings down, pushes the blocks into the goal, and finally turns and ends the code.
 */
void leftsideauto()
{
	//set initial position
	chassis.setPose(-16, -51,-90);
	//move between long goal and loader
	chassis.moveToPoint(-48,-48,1900,{.maxSpeed=120}, true);
	chassis.waitUntilDone();
	//turn to face loader
		
	chassis.turnToHeading(-180, 450);

	chassis.waitUntilDone();
	//extend tongue
	tongue.extend();
	stage1.move(105); // turn on intake (stage 1)
	pros::delay(120);
	//extract blocks from loader
	chassis.moveToPoint(-47.3, -59.3, 1000, {.maxSpeed = 127}, true);
	chassis.waitUntilDone();
	//drive to long goal
	chassis.moveToPoint(-49, -27.5, 870,{.forwards = false, .maxSpeed = 127}, true);
	chassis.waitUntilDone();
	//score the blocks
	stage1.move(127);
	stage2.move(127);
	pros::delay(1200);
	stage1.move(110);
	stage2.move(0);
	//move away from long goal
	chassis.moveToPoint(-48, -46, 860,{.forwards = true, .maxSpeed = 127}, true);
	chassis.waitUntilDone();
	//turn to face blocks, retract tongue
	tongue.retract();
	chassis.turnToHeading(45, 780);
	chassis.waitUntilDone();
	//move to blocks
	chassis.moveToPoint(-20, -19, 1400,{.maxSpeed = 70}, true);
	chassis.waitUntilDone();
	//chassis.moveToPoint(-19, -19, 1300,{.forwards = true, .maxSpeed = 70}, true);
	chassis.turnToHeading(-137, 700,{.maxSpeed = 90});
	chassis.waitUntilDone();

	//go to mid goal and score
	chassis.moveToPoint(-12, -11, 1250,{.forwards = false, .maxSpeed = 85}, true);
	//shift down block pose.
	stage1.move(-5);
	stage2.move(-50);
	chassis.waitUntilDone();
	centerupper.extend(); 
	//score mid goal
	stage1.move(100);
	stage2.move(100);
	pros::delay(1750);
	centerupper.retract();
	pros::delay(50);
	centerupper.extend();
	pros::delay(50);
	centerupper.retract();
	//move to long goal (for push) and stop rollers
	stage1.move(0);
	stage2.move(0);
	chassis.moveToPoint(-37.1, -30.5,  1700,{.forwards = true, .maxSpeed = 100}, true);
	chassis.waitUntilDone();
	//turn inside goal, wing down
	chassis.turnToHeading(0, 800);
	chassis.waitUntilDone(); 
	wing.retract();

	//push blocks into goal
	chassis.moveToPoint(-41, -9.3, 1000, {.forwards = true, .maxSpeed = 127}, true);
	//turn and end code
	chassis.turnToHeading(12, 500);
}

void soloawp()
{
	chassis.setPose(4.5,-48, -90);
	stage1.move(70); // turn on intake (stage 1)
	//push other bot
	chassis.moveToPoint(-3, -48, 900);
	//go to right loader
	chassis.moveToPoint(48.2, -48, 1500, {.forwards = false});
	//turn to face loader, extend tongue
	chassis.turnToHeading(-180, 600);
	tongue.extend();
	stage1.move(127);
	pros::delay(100);
	chassis.moveToPoint(49.5, -59.5, 800, {.maxSpeed = 127}, true); //extract blocks (first 3) from loader
	chassis.waitUntilDone();
	//move to long goal and score
	chassis.moveToPoint(49, -27, 800, {.forwards = false, .maxSpeed = 127}, true);
	chassis.waitUntilDone();
	stage1.move(127);
	stage2.move(127);
	pros::delay(700);
	//back away from long goal, reset values
	stage1.move(127);
	stage2.move(-10);
	tongue.retract();

	chassis.moveToPoint(48, -42, 1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 20}, true); 

	chassis.turnToHeading(-45, 1200);

	//move to blocks, fast

	chassis.moveToPoint(21, -18.6, 2000,{.maxSpeed = 127, .minSpeed = 35}, true);

	//turn to next batch of blocks
	//chassis.turnToHeading(-90, 600,{.maxSpeed = 110});

	chassis.moveToPoint(-18, -19.3, 1500,{ .maxSpeed = 127}, true);
	chassis.waitUntilDone();
	tongue.extend();
	//turn to face mid goal
	chassis.turnToHeading(-130, 650,{.maxSpeed = 127});
	//go to mid goal and score (reset block pose before doing that)
	chassis.moveToPoint(-11.5, -12.2, 1020,{.forwards = false, .maxSpeed = 127, .minSpeed = 127}, true);
	stage1.move(-5);
	stage2.move(-50);
	chassis.waitUntilDone(); 
	centerupper.extend(); 
	stage1.move(127);
	stage2.move(100);
	pros::delay(680);
	centerupper.retract();
	tongue.retract();
	stage1.move(0);
	stage2.move(0);

	//move to left side
	chassis.moveToPoint(-38.3, -43.8,  1300,{.forwards = true, .maxSpeed = 90, .minSpeed = 50}, true);

	chassis.turnToHeading(-180, 400);
	tongue.extend();
	chassis.waitUntilDone();

	//turn to face loader
	
	chassis.moveToPoint(-41.5, -62,  880,{.forwards = true, .maxSpeed = 127, .minSpeed = 90}, true);
	tongue.extend();
	//get blocks from loader
	stage1.move(127);
	chassis.waitUntilDone();
		//score long goal
	chassis.moveToPoint(-41.4, -33, 700, {.forwards = false, .maxSpeed = 127, .minSpeed = 110}, true);
	chassis.waitUntilDone();
	stage1.move(127);
	stage2.move(127);	

	
	

}

void skillsAuto()
{
	//starting position
	chassis.setPose(0,-48,0);
	chassis.moveToPoint(0, -43, 500, {.maxSpeed = 100}, true); // move forward a bit. 
	chassis.waitUntilDone();
	chassis.turnToHeading(-55, 600); //turn to face first red block
	chassis.waitUntilDone();
	stage1.move(70);
	chassis.moveToPoint(-26, -20.5, 1200, {.maxSpeed = 100}, true);//intake red block
	chassis.turnToHeading(-140,600);
	chassis.moveToPoint(-10, -9.5, 1250,{.forwards = false, .maxSpeed = 85}, true); //score mid goal
	stage1.move(-5);
	stage2.move(-50);
	chassis.waitUntilDone(); 
	centerupper.extend(); 
	stage1.move(127);
	stage2.move(100);
	pros::delay(800);
	stage1.move(0);
	stage2.move(0);
	chassis.moveToPoint(-43, -43.8,  1300,{.forwards = true, .maxSpeed = 90, .minSpeed = 50}, true);
	tongue.extend();
	chassis.turnToHeading(-180, 400);
	chassis.waitUntilDone();
	//get loader

	stage1.move(127);
	chassis.moveToPoint(-43, -58,  1000,{.forwards = true, .maxSpeed = 127, .minSpeed = 90}, true);
	chassis.waitUntilDone();
	pros::delay(1000);
	stage1.move(0);
	//back away from loader
	chassis.moveToPoint(-43, -40,  1000,{.forwards = false, .maxSpeed = 127}, true);
	//turn towards wall
	chassis.turnToHeading(-270, 600);




	
}

void autonomous() {
	skillsAuto();
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
			stage2.brake(); //prevents stage 2 from moving and "spitting" blocks out
		}
		else if (master.get_digital(DIGITAL_R2)){
			stage1.move(-127); //reverse stage 1;pushes blocks into low goal/unjam
			stage2.brake(); //prevents stage 2 from moving and "spitting" blocks out
		}
		//outtake
		else if (master.get_digital(DIGITAL_L1)){
			stage1.move(127); //Moves both motors to outtake blocks
			stage2.move(127);
	
		}
		else if (master.get_digital(DIGITAL_L2)){
			stage1.move(127); //75
			stage2.move(127);//70
			centerupper.extend();

		}
		else if (master.get_digital(DIGITAL_DOWN)){
			stage1.move(-25);
			stage2.move(-60);
		}
	
		else{
			stage1.move(0);
			stage2.move(0);
			centerupper.retract(); // retracts the blocker for long goal scoring
		}
		//pneumatics
		if(master.get_digital_new_press(DIGITAL_B))
		{
			tongue.toggle(); // toggles the scraper to collect blocks from the loader
		}
		if(master.get_digital(DIGITAL_Y)){
			wing.retract(); // retracts wing de-score
		}
		else{
			wing.extend(); // extends wing de-score
		}
		if (master.get_digital_new_press(DIGITAL_RIGHT)){
			centerupperdescore.toggle(); // toggles mid-goal descore mechanism
		}
		
			
		pros::delay(20);                              
	}
}
