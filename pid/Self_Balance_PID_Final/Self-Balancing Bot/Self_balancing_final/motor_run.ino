void motor_init()
{
  pinMode(8, OUTPUT);               //IN1
  pinMode(6, OUTPUT);               //pwm1
  pinMode(7, OUTPUT);               //IN2
  pinMode(10, OUTPUT);            //pwm2
  pinMode(12, OUTPUT);             //IN1
  pinMode(13, OUTPUT);              //IN2
}
void move_st()
{
  digitalWrite(8, HIGH);
  digitalWrite(7, LOW);
  analogWrite(6, 70);
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);
  analogWrite(10, 70);
}

void self_bal()
{
  if (sum > thre_pitch + 1 && sum < 30.0)
  {
    //wheelsgobackward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    final_pwm1 = 50 + (int)constrain((correction), 0, 60);
    analogWrite(pwm1, final_pwm1);
    final_pwm2 = 30 + (int)constrain((correction), 0, 60);
    analogWrite(pwm2, final_pwm2);
  }
  else if (sum < thre_pitch - 1 && sum > -30.0)

  {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    final_pwm1 = 50 + (int)(constrain(-correction, 0, 60));
    analogWrite(pwm1, final_pwm1);
    final_pwm2 =30 + (int)(constrain(-correction, 0, 60));
    analogWrite(pwm2, final_pwm2);

  }

  else if (sum > 30 && sum < 50)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    final_pwm1 = 50 + (int)constrain((correction), 0, 60);
    analogWrite(pwm1, final_pwm1);
    final_pwm2 = 30 + (int)constrain((correction), 0, 60);
    analogWrite(pwm2, final_pwm2);

  }

  else if (sum > -50 && sum < -30)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    final_pwm1 = 50 + (int)constrain((correction), 0, 60);
    analogWrite(pwm1, final_pwm1);
    final_pwm2 = 30 + (int)constrain((correction), 0, 60);
    analogWrite(pwm2, final_pwm2);

  }

  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
}

