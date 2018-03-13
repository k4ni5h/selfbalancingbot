/*
  Blink
 
  Turns an LED on for one second, then off for one second, repeatedly.
 
  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products
 
  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman
 
  This example code is in the public domain.
 
  http://www.arduino.cc/en/Tutorial/Blink
*/
 
// the setup function runs once when you press reset or power the board
double one_degree = 0.0174532;
double six_degrees = 0.1047192;
double twelve_degrees = 0.2094384;
double fifty_degrees = 0.87266;
 
const int n_states = 162;         // 3x3x6x3 = 162 states
double alpha = 1000;          // learning rate for action weights
double beta = 0.5;            // learning rate for critic weights
double gamma = 0.95;          // discount factor for critic
double lambda_w = 0.9;    // decay rate for action weights
double lambda_v = 0.8;    // decay rate for critic weights
double max_failures = 50;
double max_steps = 1000000;
 
double max_distance = 2.4;
double max_speed = 1;
double max_angle = 12 * one_degree;
 
double action_weights[n_states] = {};    // action weights
double critic_weights[n_states] = {};    // critic weights
double action_weights_elig[n_states] = {};    // action weight eligibilities
double critic_weights_elig[n_states] = {}; // critic weight eligibilities
 
 
//position, velocity, angle, angle velocity
double x = 0;
double dx = 0;
double t = 0;
double dt = 0;
 
 
 
 
int get_state()
{
    int state = 0;
 
    // failed
    if (x < -max_distance or x > max_distance or t < -max_angle or t > max_angle)
    {
      return(-1);
    }
 
    //position
    if (x < -0.8)
      state = 0;
    else if (x < 0.8)
      state = 1;
    else
      state = 2;
 
    //velocity
    if (dx < -max_speed)
      state += 0;
    else if (dx < 0.5)
      state += 3;
    else
      state += 6;
 
    //angle
    if (t < -six_degrees)
      state += 0;
    else if (t < -one_degree)
      state += 9;
    else if (t < 0)
      state += 18;
    else if (t < one_degree)
      state += 27;
    else if (t < six_degrees)
      state += 36;
    else
      state += 45;
 
    //angle velocity
    if (dt < -fifty_degrees)
      state += 0;
    else if (dt < fifty_degrees)
      state += 54;
    else
      state += 108;
 
    return(state);
}
 
 
  // read the variables x, dx, t and dt from vrep
int read_variables()
{
//    x = controller.get_current_position()[0];
//    dx = controller.get_current_ground_speed()[0];
//    t = controller.get_current_angle()[1];
//    dt = controller.get_current_angle_speed()[1];
}
 
 
 
  // executes action and updates x, dx, t, dt
  // action size is two) left and right
int do_action(bool action)
{
    //if (action == True)
    //  controller.set_target_velocities(max_speed,max_speed);
    //else
    //  controller.set_target_velocities(-max_speed,-max_speed);
}
 
 
 
  // update all weights or reset them when failed
int update_all_weights(double rhat,bool failed)
{
    for (int i=0; i<n_states; i++)
    {
        action_weights[i] += alpha * rhat * action_weights_elig[i];
        critic_weights[i] += beta * rhat * critic_weights_elig[i];
 
        if (critic_weights[i] < -1.0)
          critic_weights[i] = critic_weights[i];
 
        if (failed == true)
        {
          action_weights_elig[i] = 0;
          critic_weights_elig[i] = 0;
        }
        else
        {
          action_weights_elig[i] = action_weights_elig[i] * lambda_w;
          critic_weights_elig[i] = critic_weights_elig[i] * lambda_v;
        }
    }
   
}
 
 
 
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
 
}
 
// the loop function runs over and over again forever
void loop()
{
 
}
