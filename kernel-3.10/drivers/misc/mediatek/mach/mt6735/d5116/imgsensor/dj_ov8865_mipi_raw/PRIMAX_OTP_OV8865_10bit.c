/*
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function
bool dj_otp_update_wb()
and
bool dj_otp_update_lenc(void)
and
then the calibration of AWB & LSC & BLC will be applied. 
After finishing the OTP written, we will provide you the typical value of golden sample.
*/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
	
#include "dj_ov8865mipiraw_Sensor.h"
//#include "ov8865mipiraw_Camera_Sensor_para.h"
//#include "ov8865mipiraw_CameraCustomized.h"
	

extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
	
#define OV8865_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, 0x20)


//extern kal_uint16 OV8865_write_cmos_sensor(kal_uint16 addr, kal_uint32 para);

extern kal_uint16 DJ_OV8865_read_cmos_sensor(kal_uint32 addr);



//#define SUPPORT_FLOATING

#define OTP_LOAD_ADDR         0x3D81
#define OTP_BANK_ADDR         0x3D84

#define LENC_START_ADDR       0x5800
#define LENC_REG_SIZE         62
			
#define OTP_LENC_GROUP_FLAG   0x703A
#define OTP_LENC_GROUP_ADDR   0x703B

#define OTP_BASIC_GROUP_FLAG         0x7010
#define OTP_BASIC_GROUP_ADDR         0x7011
#define OTP_WB_GROUP_FLAG            0x7020
#define OTP_WB_GROUP_ADDR            0x7021

#define OTP_H_START_ADDR     0x3D88
#define OTP_L_START_ADDR     0x3D89
#define OTP_H_END_ADDR       0x3D8A
#define OTP_L_END_ADDR       0x3D8B
#define OTP_GROUP_SIZE       5

#define GAIN_RH_ADDR          0x5018
#define GAIN_RL_ADDR          0x5019
#define GAIN_GH_ADDR          0x501A
#define GAIN_GL_ADDR          0x501B
#define GAIN_BH_ADDR          0x501C
#define GAIN_BL_ADDR          0x501D

#define GAIN_DEFAULT_VALUE    0x0400 // 1x gain

#define OTP_MID               0x17
#define TRULY_TYPICAL_RG      0x113
#define TRULY_TYPICAL_BG      0x113




// R/G and B/G of current camera module
unsigned short dj_rg_ratio = 0;
unsigned short dj_bg_ratio = 0;
unsigned short dj_golden_rg = 0;
unsigned short dj_golden_bg = 0;
unsigned char dj_otp_lenc_data[LENC_REG_SIZE];
void DJ_DPCFuncEnable();
void DJ_DPCFuncDisable();
// Enable OTP read function
void dj_otp_read_enable(void)
{
	OV8865_write_cmos_sensor(OTP_LOAD_ADDR, 0x01);
	mdelay(5); // 
}

// Disable OTP read function
void dj_otp_read_disable(void)
{
	OV8865_write_cmos_sensor(OTP_LOAD_ADDR, 0x00);
	mdelay(5); //
}

void dj_otp_read(unsigned short otp_addr, unsigned char* otp_data)
{
	dj_otp_read_enable();
	*otp_data = DJ_OV8865_read_cmos_sensor(otp_addr);
	dj_otp_read_disable();
}

/*******************************************************************************
* Function    :  dj_otp_clear
* Description :  Clear OTP buffer 
* Parameters  :  none
* Return      :  none
*******************************************************************************/	
void dj_otp_clear(unsigned short star, unsigned short end)
{
	unsigned short i;
	// After read/write operation, the OTP buffer should be cleared to avoid accident write
	for ( i=star; i<end; i++) 
	{
		OV8865_write_cmos_sensor(i, 0x00);
	}
	
}

/*******************************************************************************
* Function    :  dj_otp_check_wb_group
* Description :  Check OTP Space Availability
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  0, group index is empty
                 1, group index has invalid data
                 2, group index has valid data
                -1, group index error
*******************************************************************************/	
signed char dj_otp_check_wb_group(int index)
{   
	unsigned char  flagWB;

    if (index > 2)
	{
		printk("OTP input wb group index %d error\n", index);
		return -1;
	}

	DJ_DPCFuncDisable();	
  
  
  
  //--------------------------------Primax---------------------------
	// select base information flag
//    unsigned short otp_addr  = OTP_BASIC_GROUP_FLAG;
//	OV8865_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
//    OV8865_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
//	OV8865_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
//	OV8865_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
//	OV8865_write_cmos_sensor(OTP_L_END_ADDR, otp_addr & 0xff);
//	msleep(5);
//   dj_otp_read(otp_addr, &flagBasic);
//	OV8865_write_cmos_sensor(otp_addr, 0x00);
//--------------------------------Primax---------------------------


	////////select AWB flag
	unsigned short otp_addr = OTP_WB_GROUP_FLAG;
	OV8865_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    OV8865_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	OV8865_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_END_ADDR, otp_addr & 0xff);
	msleep(5);
    dj_otp_read(otp_addr, &flagWB);
	OV8865_write_cmos_sensor(otp_addr, 0x00);
	DJ_DPCFuncEnable();

	// Check all bytes of a group. If all bytes are '0', then the group is empty. 
	// Check from group 1 to group 2, then group 3.
	
    if (index==0)
	{

//		flagBasic = (flagBasic>>6) & 0x03;
		flagWB = (flagWB>>2) & 0x03;
		if (!flagWB)
		{
			printk("wb group %d is empty", index);
			return 0;
		}
		else if (flagWB == 0x01 )
		{
			printk("wb group %d has valid data", index);;
			return 2;
		}
		else //if (flagBasic == 0x11)
		{
			printk("wb group %d has invalid data", index);
			return 1;
		}
	}

	else if (index == 1)
	{

	//	flagBasic=(flagBasic>>4) & 0x03;
		flagWB=(flagWB>>4) & 0x03;
		
		if (!flagWB)
		{
			printk("wb group %d is empty", index);
			return 0;
		}
		else if (flagWB == 0x01)
		{
			printk("wb group %d has valid data", index);
			return 2;
		}
		else //if (flagBasic == 0x11)
		{
			printk("wb group %d has invalid data", index);
			return 1;
		}


	}

	else
	{

//		flagBasic=(flagBasic>>2) & 0x03;
		flagWB=(flagWB>>6) & 0x03;
		
		if (!flagWB)
		{
			printk("wb group %d is empty", index);
			return 0;
		}
		else if (flagWB == 0x01)
		{
			printk("wb group %d has valid data", index);
			return 2;
		}
		else //if (flagBasic == 0x11)
		{
			printk("wb group %d has invalid data", index);
			return 1;
		}
	
	
	}
}

/*******************************************************************************
* Function    :  dj_otp_read_wb_group
* Description :  Read group value and store it in OTP Struct 
* Parameters  :  [in] index : index of otp group (0, 1, 2)
* Return      :  group index (0, 1, 2)
                 -1, error
*******************************************************************************/	
signed char dj_otp_read_wb_group()
{
	unsigned char  mid, AWB_light_LSB, dj_rg_ratio_MSB, dj_bg_ratio_MSB; 
  unsigned char  flagBasic,tempflagBasic;
	// Check first OTP with valid data
	int index =0;
  int index_basci=-1;
	for (index=0; index<3; index++)
	{
		if (dj_otp_check_wb_group(index) == 2)
		{
			printk("read wb from group %d\n", index+1);
			break;
		}
	}

	if (index > 2)
	{
		printk("no group has valid data\n");
		return -1;
	}
  		
	// ---------------------------------------Primax---------------------------
    DJ_DPCFuncDisable();
    unsigned short otp_addr_basic  = OTP_BASIC_GROUP_FLAG;
	  OV8865_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    OV8865_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr_basic>>8) & 0xff);
	  OV8865_write_cmos_sensor(OTP_L_START_ADDR, otp_addr_basic & 0xff);
	  OV8865_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr_basic>>8) & 0xff);
	  OV8865_write_cmos_sensor(OTP_L_END_ADDR, otp_addr_basic & 0xff);
	  msleep(5);
    dj_otp_read(otp_addr_basic, &flagBasic);
	  OV8865_write_cmos_sensor(otp_addr_basic, 0x00);
    DJ_DPCFuncEnable();
   
      int index_basic=-1;
      tempflagBasic=flagBasic>>2;
      if((tempflagBasic & 0x03) == 0x01)
      { 
        index_basic=0;
        printk("read basic from group %d\n", index_basic+1);
        tempflagBasic=flagBasic>>4;
      }      
      if((tempflagBasic & 0x03) == 0x01)
      {
        index_basic=1;
        printk("read basic from group %d\n", index_basic+1);
        tempflagBasic=flagBasic>>6;
      }    

      if((tempflagBasic & 0x03) == 0x01)
      {
        index_basic=2;
        printk("read basic from group %d\n", index_basic+1);
      }
      if(index_basic==-1)
      {
        printk("Primax:no valid data in basic group\n");
      }
  

// ---------------------------------------Primax-----------------------------  
	DJ_DPCFuncDisable();
	// select adress
	unsigned short otp_addr = OTP_BASIC_GROUP_ADDR + index_basci * OTP_GROUP_SIZE;
	unsigned short otp_addr1 = OTP_WB_GROUP_ADDR + index * OTP_GROUP_SIZE;

	OV8865_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    OV8865_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	OV8865_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_END_ADDR, (otp_addr+4) & 0xff);
	mdelay(5);
	dj_otp_read(otp_addr, &mid);
	OV8865_write_cmos_sensor(otp_addr, 0x00);
	
	printk("This Module ID is 0x%x\n",mid);

//	if (mid == OTP_MID)
//	{
		dj_golden_rg = TRULY_TYPICAL_RG;
		dj_golden_bg = TRULY_TYPICAL_BG;
		printk("This Module is Truly Module\n");
    printk("This Module ID is %x\n",mid);
//	}
	/*else
	{
		printk("This Module is Other Module\n");
		//ÆäËû²Ù×÷
		return -1;
	}  */

	OV8865_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    OV8865_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr1>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_START_ADDR, otp_addr1 & 0xff);
	OV8865_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr1>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_END_ADDR, (otp_addr1+4) & 0xff);
	mdelay(5);
	dj_otp_read(otp_addr1,  &dj_rg_ratio_MSB);
	dj_otp_read(otp_addr1+1,  &dj_bg_ratio_MSB);
	dj_otp_read(otp_addr1+4, &AWB_light_LSB);	
	dj_otp_clear(otp_addr1,otp_addr1+OTP_GROUP_SIZE);
	DJ_DPCFuncEnable();
	dj_rg_ratio = (dj_rg_ratio_MSB<<2) | ((AWB_light_LSB & 0xC0)>>6);
	dj_bg_ratio = (dj_bg_ratio_MSB<<2) | ((AWB_light_LSB & 0x30)>>4);
	printk("dj_rg_ratio=0x%x, dj_bg_ratio=0x%x\n", dj_rg_ratio, dj_bg_ratio);
	printk("read wb finished\n");
	return index;
}

#ifdef SUPPORT_FLOATING //Use this if support floating point values
/*******************************************************************************
* Function    :  dj_otp_apply_wb
* Description :  Calcualte and apply R, G, B gain to module
* Parameters  :  [in] dj_golden_rg : R/G of golden camera module
                 [in] dj_golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool dj_otp_apply_wb()
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	double ratio_r, ratio_g, ratio_b;
	double cmp_rg, cmp_bg;

	if (!dj_golden_rg || !dj_golden_bg)
	{
		printk("dj_golden_rg / dj_golden_bg can not be zero\n");
		return 0;
	}
	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
        // and R/G, B/G of current module
	cmp_rg = 1.0 * dj_rg_ratio / dj_golden_rg;
	cmp_bg = 1.0 * dj_bg_ratio / dj_golden_bg;

	if ((cmp_rg<1) && (cmp_bg<1))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1;
		ratio_r = 1 / cmp_rg;
		ratio_b = 1 / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1;
		ratio_g = cmp_rg;
		ratio_b = cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1;
		ratio_g = cmp_bg;
		ratio_r = cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1)
	{
		gain_r = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_r);
		OV8865_write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 6);
		OV8865_write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x003f);
	}

	if (ratio_g != 1)
	{
		gain_g = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_g);
		OV8865_write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 6);
		OV8865_write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x003f);
	}

	if (ratio_b != 1)
	{
		gain_b = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_b);
		OV8865_write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 6);
		OV8865_write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x003f);
	}

	printk("cmp_rg=%f, cmp_bg=%f\n", cmp_rg, cmp_bg);
	printk("ratio_r=%f, ratio_g=%f, ratio_b=%f\n", ratio_r, ratio_g, ratio_b);
	printk("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}

#else //Use this if not support floating point values

#define OTP_MULTIPLE_FAC	10000
bool dj_otp_apply_wb()
{
	unsigned short gain_r = GAIN_DEFAULT_VALUE;
	unsigned short gain_g = GAIN_DEFAULT_VALUE;
	unsigned short gain_b = GAIN_DEFAULT_VALUE;

	unsigned short ratio_r, ratio_g, ratio_b;
	unsigned short cmp_rg, cmp_bg;

	if (!dj_golden_rg || !dj_golden_bg)
	{
		printk("dj_golden_rg / dj_golden_bg can not be zero\n");
		return 0;
	}

	// Calcualte R, G, B gain of current module from R/G, B/G of golden module
    // and R/G, B/G of current module
	cmp_rg = OTP_MULTIPLE_FAC * dj_rg_ratio / dj_golden_rg;
	cmp_bg = OTP_MULTIPLE_FAC * dj_bg_ratio / dj_golden_bg;

	if ((cmp_rg < 1 * OTP_MULTIPLE_FAC) && (cmp_bg < 1 * OTP_MULTIPLE_FAC))
	{
		// R/G < R/G golden, B/G < B/G golden
		ratio_g = 1 * OTP_MULTIPLE_FAC;
		ratio_r = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_rg;
		ratio_b = 1 * OTP_MULTIPLE_FAC * OTP_MULTIPLE_FAC / cmp_bg;
	}
	else if (cmp_rg > cmp_bg)
	{
		// R/G >= R/G golden, B/G < B/G golden
		// R/G >= R/G golden, B/G >= B/G golden
		ratio_r = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_rg;
		ratio_b = OTP_MULTIPLE_FAC * cmp_rg / cmp_bg;
	}
	else
	{
		// B/G >= B/G golden, R/G < R/G golden
		// B/G >= B/G golden, R/G >= R/G golden
		ratio_b = 1 * OTP_MULTIPLE_FAC;
		ratio_g = cmp_bg;
		ratio_r = OTP_MULTIPLE_FAC * cmp_bg / cmp_rg;
	}

	// write sensor wb gain to registers
	// 0x0400 = 1x gain
	if (ratio_r != 1 * OTP_MULTIPLE_FAC)
	{
		gain_r = GAIN_DEFAULT_VALUE * ratio_r / OTP_MULTIPLE_FAC;
		OV8865_write_cmos_sensor(GAIN_RH_ADDR, gain_r >> 6);
		OV8865_write_cmos_sensor(GAIN_RL_ADDR, gain_r & 0x003f);
	}

	if (ratio_g != 1 * OTP_MULTIPLE_FAC)
	{
		gain_g = GAIN_DEFAULT_VALUE * ratio_g / OTP_MULTIPLE_FAC;
		OV8865_write_cmos_sensor(GAIN_GH_ADDR, gain_g >> 6);
		OV8865_write_cmos_sensor(GAIN_GL_ADDR, gain_g & 0x003f);
	}

	if (ratio_b != 1 * OTP_MULTIPLE_FAC)
	{
		gain_b = (unsigned short)(GAIN_DEFAULT_VALUE * ratio_b/ OTP_MULTIPLE_FAC);
		OV8865_write_cmos_sensor(GAIN_BH_ADDR, gain_b >> 6);
		OV8865_write_cmos_sensor(GAIN_BL_ADDR, gain_b & 0x003f);
	}

	printk("cmp_rg=%d, cmp_bg=%d\n", cmp_rg, cmp_bg);
	printk("ratio_r=%d, ratio_g=%d, ratio_b=%d\n", ratio_r, ratio_g, ratio_b);
	printk("gain_r=0x%x, gain_g=0x%x, gain_b=0x%x\n", gain_r, gain_g, gain_b);
	return 1;
}
#endif /* SUPPORT_FLOATING */

/*******************************************************************************
* Function    :  dj_otp_update_wb
* Description :  Update white balance settings from OTP
* Parameters  :  void
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool dj_otp_update_wb() 
{
	printk("start wb update\n");

	if (dj_otp_read_wb_group() != -1)
	{
		if (dj_otp_apply_wb() == 1)
		{
			printk("wb update finished\n");
			return 1;
		}
	}
	printk("wb update failed\n");
	return 0;
}

/*******************************************************************************
* Function    :  dj_otp_check_lenc_group
* Description :  Check OTP Space Availability
* Parameters  :  [in] int  index : index of otp group (0, 1, 2)
* Return      :  0, group index is empty
                 1, group index has invalid data
                 2, group index has valid data
                -1, group index error
*******************************************************************************/	
signed char dj_otp_check_lenc_group(int  index)
{   
	unsigned char  flag;

    if (index > 2)
	{
		printk("OTP input lenc group index %d error\n", index);
		return -1;
	}
    DJ_DPCFuncDisable();
	// select lenc flag
    unsigned short otp_addr = OTP_LENC_GROUP_FLAG;
	OV8865_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    OV8865_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	OV8865_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_END_ADDR, otp_addr & 0xff);
	msleep(5);
    dj_otp_read(otp_addr, &flag);
	OV8865_write_cmos_sensor(otp_addr, 0x00);
	DJ_DPCFuncEnable();

	// Check all bytes of a group. If all bytes are '0', then the group is empty. 
	// Check from group 1 to group 2, then group 3.
	if (index==0)
	{
      flag=(flag>>2) & 0x03;

	  if (!flag)
	  {
		  printk("lenc group %d is empty", index+1);
		  return 0;
	  }
	  else if (flag == 0x01)
	  {
		  printk("lenc group %d has valid data", index+1);
		  return 2;
	  }
	  else //if (flag == 0x11)
	  {
		  printk("lenc group %d has invalid data", index)+1;
		  return 1;
	  }
	}

	else if (index == 1)
	{

		flag=(flag>>4) & 0x03;
		
		if (!flag)
		{
			printk("lenc group %d is empty", index+1);
			return 0;
		}
		else if (flag == 0x01)
		{
			printk("lenc group %d has valid data", index+1);
			return 2;
		}
		else //if (flag == 0x11)
		{
			printk("lenc group %d has invalid data", index+1);
			return 1;
		}


	}

	else
	{

		flag=(flag>>6) & 0x03;
		
		if (!flag)
		{
			printk("lenc group %d is empty", index+1);
			return 0;
		}
		else if (flag == 0x01)
		{
			printk("lenc group %d has valid data", index+1);
			return 2;
		}
		else //if (flag == 0x11)
		{
			printk("lenc group %d has invalid data", index+1);
			return 1;
		}
	
	
	}
}

/*******************************************************************************
* Function    :  dj_otp_read_lenc_group
* Description :  Read group value and store it in OTP Struct 
* Parameters  :  [in] int index : index of otp group (0, 1, 2)
* Return      :  group index (0, 1, 2)
                 -1, error
*******************************************************************************/	
signed char dj_otp_read_lenc_group()
{
	int index =0;
	for (index=0; index<3; index++)
	{
		if (dj_otp_check_lenc_group(index) == 2)
		{
			printk("read lenc from group %d\n", index+1);
			break;
		}
	}

	if (index > 2)
	{
		printk("no group has valid data\n");
		return -1;
	}
	DJ_DPCFuncDisable();

	// read lenc data
	unsigned short otp_addr = OTP_LENC_GROUP_ADDR + index * LENC_REG_SIZE;
	OV8865_write_cmos_sensor(OTP_BANK_ADDR, 0xc0);
    OV8865_write_cmos_sensor(OTP_H_START_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_START_ADDR, otp_addr & 0xff);
	OV8865_write_cmos_sensor(OTP_H_END_ADDR, (otp_addr>>8) & 0xff);
	OV8865_write_cmos_sensor(OTP_L_END_ADDR, (otp_addr+LENC_REG_SIZE) & 0xff);
	dj_otp_read_enable();
	int i=0;
	for(i=0; i<LENC_REG_SIZE; i++) 
	{
		dj_otp_lenc_data[i] = DJ_OV8865_read_cmos_sensor(otp_addr);
		otp_addr++;
	}
	dj_otp_read_disable();
	dj_otp_clear(otp_addr,otp_addr+LENC_REG_SIZE);
	DJ_DPCFuncEnable();
	
	printk("read lenc finished\n");
	return index;
}

/*******************************************************************************
* Function    :  dj_otp_apply_lenc
* Description :  Apply lens correction setting to module
* Parameters  :  none
* Return      :  none
*******************************************************************************/	
void dj_otp_apply_lenc(void)
{
	// write lens correction setting to registers
	printk("apply lenc setting\n");
	int i=0;
	for(i=0; i<LENC_REG_SIZE; i++)
	{
		OV8865_write_cmos_sensor(LENC_START_ADDR+i, dj_otp_lenc_data[i]);
		printk("0x%x, 0x%x\n", LENC_START_ADDR+i, dj_otp_lenc_data[i]);
	}
	
	//Enable LSC
	unsigned char temp = DJ_OV8865_read_cmos_sensor(0x5000);
	temp = temp | 0x90;
	OV8865_write_cmos_sensor(0x5000, temp);
	printk("LSC_Enable:0x%x\n", temp);
}

/*******************************************************************************
* Function    :  dj_otp_update_lenc
* Description :  Get lens correction setting from otp, then apply to module
* Parameters  :  none
* Return      :  1, success; 0, fail
*******************************************************************************/	
bool dj_otp_update_lenc(void) 
{
	printk("start lenc update\n");

	if (dj_otp_read_lenc_group() != -1)
	{
		dj_otp_apply_lenc();
		printk("lenc update finished\n");
		return 1;
	}

	printk("lenc update failed\n");
	return 0;
}
/*****************************************************************************
*Function    :  DPC Function
*Description :  To avoid  OTP memory R/W error 
 Before doing OTP read/write,register 0x5002[3] must be set to ¡°0¡±. 
 After OTP memory access,set register 0x5002[3] back to ¡°1¡±.
******************************************************************************/
void DJ_DPCFuncEnable()
{
	unsigned short ctrl;
	ctrl = DJ_OV8865_read_cmos_sensor(0x5002);
	ctrl = ctrl | 0x08;
	OV8865_write_cmos_sensor(0x5002,ctrl);
}
void DJ_DPCFuncDisable()
{
	unsigned short ctrl;
	ctrl = DJ_OV8865_read_cmos_sensor(0x5002);
	ctrl = ctrl & (~0x08);
	OV8865_write_cmos_sensor(0x5002,ctrl);

}

