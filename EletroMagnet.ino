

// 引脚定义
const int IR_In = A0;  // 红外输入引脚
const int EleMagOutPin = 9; //  电磁铁PWM输入出引脚

// 变量定义 
int sensorValue = 0;        // 电位器电压值, 读取红外数值
int outputValue = 0;        // 模拟量输出值（PWM）,输出PWM
float distance = 0;    //传感器到浮子的距离
float height = 0;   // 传感器到电磁铁高度
float x = 0;    //电磁铁到浮子距离
float u0 = 0;     // 电压初始值，需要改(0-1023)
float x0 = 0;     // 距离初始值，需要改
float P = 0; 
float I = 0;
float D = 0;
float error = 0;
float last_error = 0;
float diff_error= 0;
float integer_error = 0;

// 用于标定传感器到电磁铁高度
const int COUNT = 500;
float h_tmp = 0; 
int count= 0;
bool calibFinish = false;

void setup() {
  // 初始化串口参数
  Serial.begin(9600); 
}

int PIDController(float error,float diff_error,float integer_error)
{
  int output;
  float delta_u = P*error + D*diff_error + I*integer_error;
  float u = u0 + delta_u;
  if(u > 255)
    u = 255;
  if(u < 0)
    u = 0;
  output = (int)(u);
  return output;  
}

float CalDistance(int sensorValue)
{
  double b = -0.0078;
  double a = 0.0002;
  float dis = 1/(a*sensorValue + b +0.0001);
  return dis;
}

void loop() {
  // 标定传感器到电磁铁距离
  if(!calibFinish)
  {
      sensorValue = analogRead(IR_In); 
      distance = CalDistance(sensorValue);
      h_tmp+=distance;
  }

  if(calibFinish)
  {
	  sensorValue = analogRead(IR_In); 
	  distance = CalDistance(sensorValue);
	  x = height - distance;
	  error = x - x0;
	  diff_error = error - last_error;
	  integer_error = integer_error + error;
	  outputValue = PIDController(error, diff_error, integer_error);
	  analogWrite(EleMagOutPin, outputValue);           

	  Serial.print("x = ");
	  Serial.print(x);
	  Serial.print("cm    ");
	  Serial.print("u = ");
	  Serial.print(outputValue);
	  Serial.print("\n");
	  
	  last_error = error;
  }

  if(!calibFinish)
  {
      count++;
      if(count >= COUNT)
      {
          calibFinish = true;
          height = h_tmp/COUNT;
          Serial.print("Calibration Finished, Height is " );
          Serial.print(height);
          Serial.print("cm. \n"); 
      }
  }

  delay(2);                     
}
