#include <Servo.h>

Servo f, m, r;                          /* Front, middle and rear servos */
int fa, ma, ra;                         /* Front, middle and rear servos current angle */

int d = 10;                             /* Delay between 1° step in milliseconds */
int s_center = 90;                      /* Servos default angle */
int s_step = 1;                         /* Angle step when moving servos */

int fpin = 9, mpin = 11, rpin = 10;     /* Front, middle and rear servos digital pins */

int moving_legs_min = s_center - 20;    /* Front and rear servos minimum angle: right leg front */
int moving_legs_max = s_center + 20;    /* Front and rear servos maximum angle: left leg front */
int bending_legs_min = s_center - 20;   /* Middle servo minimum angle: bend right */
int bending_legs_max = s_center + 20;   /* Middle servo maximum angle: bend left */

void setup()
{
  fa = ma = ra = s_center;
  f.attach(fpin);
  f.write(fa);
  m.attach(mpin);
  m.write(ma);
  r.attach(rpin);
  r.write(ra);

  delay(2000);    /* Wait for 2 seconds before entering the loop */
}

/*
 * servo_move()
 *
 * Modifies the angle from <current angle> to <target angle> by <s_step> degree
 */
void servo_move(Servo *s, int *current_angle, int target_angle)
{
  int step;
  if (*current_angle == target_angle) return;
  step = *current_angle > target_angle ? -s_step : s_step;

  do {
    *current_angle += step;
    s->write(*current_angle);
    delay(d);
  } while (*current_angle != target_angle);
}

/*
 * distance_to_expected_position()
 *
 * Computes the cost of the planned position from current position
 */
int distance_to_planned_position(int expected_fa, int expected_ma, int expected_ra)
{
  if ((expected_fa != fa) || (expected_ra != ra)) {
    /* 4 operations to perform:
     * - move the bending leg to middle
     * - move the front leg to expected position
     * - move the rear leg to expected position
     * - move the bending leg to expected position
     */
    return abs(s_center - ma) + abs(expected_fa - fa) + abs(expected_ra - ra) +
      abs(expected_ma - s_center);
  } else {
    return abs(expected_ma - ma);
  }
}

#define f_move(target_angle)  servo_move(&f, &fa, target_angle)
#define m_move(target_angle)  servo_move(&m, &ma, target_angle)
#define r_move(target_angle)  servo_move(&r, &ra, target_angle)

#define bend_left_angle   bending_legs_max
#define bend_right_angle  bending_legs_min

#define bend_left()    m_move(bend_left_angle)
#define bend_right()   m_move(bend_right_angle)
#define bend_middle()  m_move(s_center)

#define moving_left_leg_tofront_angle   moving_legs_max
#define moving_right_leg_tofront_angle  moving_legs_min
/* Aliases to above for clarity */
#define moving_left_leg_torear_angle    moving_right_leg_tofront_angle
#define moving_right_leg_torear_angle   moving_left_leg_tofront_angle

#define move_left_front_leg_tofront()  f_move(moving_left_leg_tofront_angle)
#define move_left_front_leg_torear()   f_move(moving_left_leg_torear_angle)
#define move_right_front_leg_tofront() move_left_front_leg_torear()
#define move_right_front_leg_torear()  move_left_front_leg_tofront()

#define move_left_rear_leg_tofront()   r_move(moving_left_leg_tofront_angle)
#define move_left_rear_leg_torear()    r_move(moving_left_leg_torear_angle)
#define move_right_rear_leg_tofront()  move_left_rear_leg_torear()
#define move_right_rear_leg_torear()   move_left_rear_leg_tofront()

void turn_left()
{
  /*
   * To turn left on itself:
   * - move right front and rear legs from front to rear (move #A)
   * - move left front and rear legs from rear to front (move #B)
   *
   * Order of the operations is chosen based on current position.
   *
   * Legend:
   * O  - foot touching ground
   * *  - foot not touching ground
   * XX - robot core
   *
   * Move #A:      *
   *     O          \-O
   *  *-/           XX
   *   XX     to   *-\    (moves forwards a bit)
   *   /-O            O
   *  *
   *
   * Move #B:
   *     *
   *  O-/          O
   *   XX     to    \-*   (moves backwards a bit)
   *   /-*          XX
   *  O            O-\
   *                  *
   */

  int moveA_dist = distance_to_planned_position(
      moving_right_leg_tofront_angle, bend_right_angle,
      moving_right_leg_tofront_angle);
  int moveB_dist = distance_to_planned_position(
      moving_left_leg_torear_angle, bend_left_angle,
      moving_left_leg_torear_angle);

  if (moveA_dist <= moveB_dist) {

    if (moveA_dist != 0) {
      /* Prepare for move #A */
      bend_middle();
      move_right_front_leg_tofront();
      move_right_rear_leg_tofront();
      bend_right();
    }

    /* Move #A */
    move_right_front_leg_torear();
    move_right_rear_leg_torear();

    /* Left front and rear legs are now at front, prepare for move #B */
    bend_middle();
    move_left_front_leg_torear();
    move_left_rear_leg_torear();
    bend_left();

    /* Move #B */
    move_left_front_leg_tofront();
    move_left_rear_leg_tofront();

  } else {

    if (moveB_dist != 0) {
      /* Prepare for move #B */
      bend_middle();
      move_left_front_leg_torear();
      move_left_rear_leg_torear();
      bend_left();
    }

    /* Move #B */
    move_left_front_leg_tofront();
    move_left_rear_leg_tofront();

    /* Right front and rear legs are now at rear, prepare for move #A */
    bend_middle();
    move_right_front_leg_tofront();
    move_right_rear_leg_tofront();
    bend_right();

    /* Move #A */
    move_right_front_leg_torear();
    move_right_rear_leg_torear();

  }
}

void turn_right()
{
  /*
   * To turn right on itself:
   * - move left front and rear legs from front to rear (move #A)
   * - move right front and rear legs from rear to front (move #B)
   *
   * Order of the operations is chosen based on current position.
   *
   * Legend:
   * O  - foot touching ground
   * *  - foot not touching ground
   * XX - robot core
   *
   * Move #A:         *
   *  O            O-/
   *   \-*          XX
   *   XX     to    /-*   (moves forwards a bit)
   *  O-\          O
   *     *
   *
   * Move #B:
   *  *
   *   \-O            O
   *   XX     to   *-/    (moves backwards a bit)
   *  *-\           XX
   *     O          /-O
   *               *
   */

  int moveA_dist = distance_to_planned_position(
      moving_left_leg_tofront_angle, bend_left_angle,
      moving_left_leg_tofront_angle);
  int moveB_dist = distance_to_planned_position(
      moving_right_leg_torear_angle, bend_right_angle,
      moving_right_leg_torear_angle);

  if (moveA_dist <= moveB_dist) {

    if (moveA_dist != 0) {
      /* Prepare for move #A */
      bend_middle();
      move_left_front_leg_tofront();
      move_left_rear_leg_tofront();
      bend_left();
    }

    /* Move #A */
    move_left_front_leg_torear();
    move_left_rear_leg_torear();

    /* Right front and rear legs are now at front, prepare for move #B */
    bend_middle();
    move_right_front_leg_torear();
    move_right_rear_leg_torear();
    bend_right();

    /* Move #B */
    move_right_front_leg_tofront();
    move_right_rear_leg_tofront();

  } else {

    if (moveB_dist != 0) {
      /* Prepare for move #B */
      bend_middle();
      move_right_front_leg_torear();
      move_right_rear_leg_torear();
      bend_right();
    }

    /* Move #B */
    move_right_front_leg_tofront();
    move_right_rear_leg_tofront();

    /* Left front and rear legs are now at rear, prepare for move #A */
    bend_middle();
    move_left_front_leg_tofront();
    move_left_rear_leg_tofront();
    bend_left();

    /* Move #A */
    move_left_front_leg_torear();
    move_left_rear_leg_torear();

  }
}

void go_forward()
{
  /*
   * To go forward:
   * - move the left front and rear legs from front to rear (move #A)
   * - move the right front and rear legs from front to rear (move #B)
   *
   * Order of the operations is chosen based on current position.
   *
   * Legend:
   * O  - foot touching ground
   * *  - foot not touching ground
   * XX - robot core
   *
   * Move #A:         *
   *  O            O-/
   *   \-*          XX
   *   XX     to    /-*
   *  O-\          O
   *     *
   *
   * Move #B:      *
   *     O          \-O
   *  *-/           XX
   *   XX     to   *-\
   *   /-O            O
   *  *
   */
  int moveA_dist = distance_to_planned_position(
      moving_left_leg_tofront_angle, bend_left_angle,
      moving_left_leg_tofront_angle);
  int moveB_dist = distance_to_planned_position(
      moving_right_leg_tofront_angle, bend_right_angle,
      moving_right_leg_tofront_angle);

  if (moveA_dist <= moveB_dist) {

    if (moveA_dist != 0) {
      /* Prepare for move #A */
      bend_middle();
      move_left_front_leg_tofront();
      move_left_rear_leg_tofront();
      bend_left();
    }

    /* Move #A */
    move_left_front_leg_torear();
    move_left_rear_leg_torear();

    /* Right front and rear legs are already at front, prepare for move #B */
    bend_right();

    /* Move #B */
    move_right_front_leg_torear();
    move_right_rear_leg_torear();

  } else {

    if (moveB_dist != 0) {
      /* Prepare for move #B */
      bend_middle();
      move_right_front_leg_tofront();
      move_right_rear_leg_tofront();
      bend_right();
    }

    /* Move #B */
    move_right_front_leg_torear();
    move_right_rear_leg_torear();

    /* Left front and rear legs are already at front, prepare for move #B */
    bend_left();

    /* Move #A */
    move_left_front_leg_torear();
    move_left_rear_leg_torear();

  }
}

void go_backward()
{
  /*
   * To go backward:
   * - move the left front and rear legs from rear to front (move #A)
   * - move the right front and rear legs from rear to front (move #B)
   *
   * Order of the operations is chosen based on current position.
   *
   * Legend:
   * O  - foot touching ground
   * *  - foot not touching ground
   * XX - robot core
   *
   * Move #A:
   *     *
   *  O-/           O
   *   XX     to     \-*
   *   /-*           XX
   *  O             O-\
   *                   *
   * Move #B:
   *  *
   *   \-O             O
   *   XX     to    *-/
   *  *-\            XX
   *     O           /-O
   *                *
   */
  int moveA_dist = distance_to_planned_position(
      moving_left_leg_torear_angle, bend_left_angle,
      moving_left_leg_torear_angle);
  int moveB_dist = distance_to_planned_position(
      moving_right_leg_torear_angle, bend_right_angle,
      moving_right_leg_torear_angle);

  if (moveA_dist <= moveB_dist) {

    if (moveA_dist != 0) {
      /* Prepare for move #A */
      bend_middle();
      move_left_front_leg_torear();
      move_left_rear_leg_torear();
      bend_left();
    }

    /* Move #A */
    move_left_front_leg_tofront();
    move_left_rear_leg_tofront();

    /* Right front and rear legs are already at rear, prepare for move #B */
    bend_right();

    /* Move #B */
    move_right_front_leg_tofront();
    move_right_rear_leg_tofront();

  } else {

    if (moveB_dist != 0) {
      /* Prepare for move #B */
      bend_middle();
      move_right_front_leg_torear();
      move_right_rear_leg_torear();
      bend_right();
    }

    /* Move #B */
    move_right_front_leg_tofront();
    move_right_rear_leg_tofront();

    /* Left front and rear legs are already at rear, prepare for move #B */
    bend_left();

    /* Move #A */
    move_left_front_leg_tofront();
    move_left_rear_leg_tofront();

  }
}

void loop()
{
}

/* vim: set filetype=c autoindent tabstop=2 shiftwidth=2 softtabstop=2 expandtab : */
