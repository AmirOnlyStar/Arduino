#ifndef _SERIALCAMMAND_H_
#define _SERIALCAMMAND_H_
#include "string.h"
float sp=0;

char str_command[256];
int str_command_counter = 0;

struct command read_command(char *str_in);
s8 run_command(struct command cmd);

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

  const char command_1[]="SERIAL\0";//Serial Test
  const char command_2[]="STEP:\0";//Serial Test
  const char command_3[]="ESC1:\0";//Serial Test
  const char command_4[]="ESC2:\0";//Serial Test
  const char command_5[]="STEP2:\0";//Serial Test
  const char command_6[]="SP:\0";//Serial Test
  const char command_7[]="PID:\0";//Serial Test
  const char command_8[]="MOT1:\0";//Serial Test

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
  }else if(!strncmp(str_in,command_6,strlen(command_6)))
  {
    cmd.command_id    = 6;
  }else if(!strncmp(str_in,command_7,strlen(command_7)))
  {
    cmd.command_id    = 7;
  }else if(!strncmp(str_in,command_8,strlen(command_8)))
  {
    cmd.command_id    = 8;
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
        Serial.println("Serial ready on 250000 ");
        break;
    }
    case 2:
    {
      Serial.println("Step test:");
      if(cmd.command_value1 == 0) step_disable();
      else
      {
        step_run(cmd.command_value1,cmd.command_value2);
      }

      break;
    }
    case 3:
    {
      Serial.println("ESC1 test:");
      ESC1_write(cmd.command_value1);
      break;
    }
    case 4:
    {
      Serial.println("ESC2 test:");
      ESC2_write(cmd.command_value2);
      break;
    }
    case 5:
    {
      Serial.println("Step2 test:");
      step_run2(cmd.command_value1);
      break;
    }
    case 6:
    {
      Serial.print("SP set:");
      sp = cmd.command_value1;
      Serial.println(sp);
      break;
    }
    case 7:
    {
      Serial.print("PID set:");
      PID1.PID_set_param(cmd.command_value1,cmd.command_value2,cmd.command_value3);
      break;
    }
    case 8:
    {
      Serial.print("MOT1 set:");
      mot1_setSpeed(cmd.command_value1);
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
