// #ifndef _CONTROL_H_
// #define _CONTROL_H_


class my_PID
{
  private:
  float p,i,d;
  float val_p,val_i,val_d;
  float dt;
  u32 last_time_pid;
  float last_er=0;
  public:
  void PID_set_param(float kp,float ki ,float kd);
  float PID_calculate(float er);
  void PID_reset(void);
};

void my_PID::PID_set_param(float kp,float ki ,float kd)
{
  p = kp;
  i = ki;
  d = kd;
  Serial.print("PID_SET:");
  Serial.print(p);Serial.print(" ");
  Serial.print(i);Serial.print(" ");
  Serial.print(d);Serial.print("\r\n");
}
float my_PID:: PID_calculate(float er)
{
  dt = (double)(micros() - last_time_pid) / 1000000; // Calculate delta time

  last_time_pid = micros();
  val_p = er*p;
  val_i = val_i+(er*dt)*i;

  val_d = (er-last_er)*d;
  last_er = er;

  // Serial.print("P:");
  // Serial.print(p);Serial.print("  I:");
  // Serial.print(i);Serial.print("  D:");
  // Serial.print(d);Serial.print("\t\t");
  //
  // Serial.print("VP:");
  // Serial.print(val_p);Serial.print("  VI:");
  // Serial.print(val_i);Serial.print("  VD:");
  // Serial.print(val_d);Serial.print("\t");

  return val_p+val_i+val_d;
}

void my_PID:: PID_reset(void)
{
  last_er = 0;
  val_i = 0;
}


// #endif
