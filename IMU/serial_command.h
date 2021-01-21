#ifndef _SERIALCAMMAND_H_
#define _SERIALCAMMAND_H_
#include "my_type.h"
#include "string.h"
float sp=0;

char str_command[256];
int str_command_counter = 0;

struct command read_command(char *str_in);
s8 run_command(struct command cmd);


/*************************************/
extern u8 serial_out_mode;
extern bool flag_acc_calibrarion ,flag_gyro_calibrarion,flag_mag_calibrarion;
extern void acc_manual_offset(int16_t offsetX,int16_t offsetY,int16_t offsetZ);
/**************************************/


struct command
{
u32 command_id;
float command_value1;
float command_value2;
float command_value3;
};

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n')
    {
      struct command cmd;
      // Serial.println(str_command);

      cmd = read_command(str_command);
      run_command(cmd);

      str_command_counter=0;
      memset(str_command,0,sizeof(str_command));
    }else
    {
      str_command[str_command_counter++] = inChar;
    }
  }
}


struct command read_command(char *str_in)
{
  command cmd;
  char str1[100];
  float val1,val2,val3;

  const char command_1[]="SERIAL:\0";//Serial Test
  const char command_2[]="ACC_CAL:\0";
  const char command_3[]="GYRO_CAL:\0";
  const char command_4[]="MAG_CAL:\0";
  const char command_5[]="ACC_OFFSET:\0";



  // Serial.println(str_in);
  cmd.command_id    = 0;
  cmd.command_value1 = 0;
  cmd.command_value2 = 0;
  cmd.command_value3 = 0;

  memset(str1,0,sizeof(str1));
  sscanf(str_in,"%*[^:]:%s",str1);
  val1 = atof(str1);

  memset(str1,0,sizeof(str1));
  sscanf(str_in,"%*[^:]:%*[^:]:%s",str1);
  val2 = atof(str1);
  memset(str1,0,sizeof(str1));
  sscanf(str_in,"%*[^:]:%*[^:]:%*[^:]:%s",str1);
  val3 = atof(str1);

  cmd.command_value1 = val1;
  cmd.command_value2 = val2;
  cmd.command_value3 = val3;

  if(!strncmp(str_in,command_1,strlen(command_1)))
  {
    cmd.command_id    = 1;
  //############################### #STEP ###############################
  }else if(!strncmp(str_in,command_2,strlen(command_2)))
  {
    cmd.command_id    = 2;
  //############################### #ESC1 ###############################
  }else if(!strncmp(str_in,command_3,strlen(command_3)))
  {
    cmd.command_id    = 3;
  //############################### #ESC2 ###############################
  }else if(!strncmp(str_in,command_4,strlen(command_4)))
  {
    cmd.command_id    = 4;
  //############################### #STEP2 ###############################
  }else if(!strncmp(str_in,command_5,strlen(command_5)))
  {
    cmd.command_id    = 5;
  //############################### #STEP2 ###############################
  }

  return cmd;
}

s8 run_command(struct command cmd)
{
  Serial.println("################################");
  Serial.print("command_id:");
  Serial.print(cmd.command_id);
  Serial.print("\t");
  Serial.print("command_value1:");
  Serial.print(cmd.command_value1);
  Serial.print("\t");
  Serial.print("command_value2:");
  Serial.print(cmd.command_value2);
  Serial.print("\t");
  Serial.print("command_value3:");
  Serial.println(cmd.command_value3);
  Serial.println("################################");
  switch (cmd.command_id)
  {
    case 0:
    {
      Serial.println("Invalid command!");
      break;
    }
    case 1:
    {
        Serial.println("Serial ready on 1000000");
        serial_out_mode = cmd.command_value1;
        Serial.print("Serial out mode:");Serial.println(serial_out_mode);
        break;
    }
    case 2:
    {
      Serial.println("ACC calibration:");
      flag_acc_calibrarion = cmd.command_value1;
      break;
    }
    case 3:
    {
      Serial.println("GYRO calibration:");
      flag_gyro_calibrarion = cmd.command_value1;
      break;
    }
    case 4:
    {
      Serial.println("MAG calibration:");
      flag_mag_calibrarion = cmd.command_value1;
      break;
    }
    case 5:
    {
      Serial.println("ACC Manual Offset:");
      acc_manual_offset(cmd.command_value1,cmd.command_value2,cmd.command_value3);
      break;
    }
  }//end switch
  return 0;

}
#endif
// Serial.println("***************************");
// for(i=0;i<strlen(str_command);i++)
// {
//   Serial.print(str_command[i],HEX);
//   Serial.print(" ");
// }
// Serial.println("***************************");
