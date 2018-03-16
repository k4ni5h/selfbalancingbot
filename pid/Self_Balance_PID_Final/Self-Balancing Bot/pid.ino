void pid()
{
  error=sum-thre_pitch;
  diff_error = error - prev_error;
  sum_error += error;
  if(sum_error>50)
  sum_error=0;
  correction = kp*error + kd*diff_error + ki*sum_error;
  prev_error = error;
  
 }

