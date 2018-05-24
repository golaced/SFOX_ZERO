#include "flash_on_chip.h"
#include "app_ano.h"
#include "mpu9250.h"

FLASH_EraseInitTypeDef EraseInitStruct;
static uint32_t GetSector(uint32_t Address);

uint8_t save_arry[max_flash_save_len];
short save_time;

void FLASHinit(void);

void flash_save_parameters(void)
{
	save_arry[0] = 0xff;
	save_arry[1] = 0xfe;
	save_arry[max_flash_save_len - 2] = 0xfe;
	save_arry[max_flash_save_len - 1] = 0xff;

	flash_float_to_byte(save_arry, (short)0, &(rol_p));
	flash_float_to_byte(save_arry, (short)1, &(rol_i));
	flash_float_to_byte(save_arry, (short)2, &(rol_d));

	flash_float_to_byte(save_arry, (short)3, &(pit_p));
	flash_float_to_byte(save_arry, (short)4, &(pit_i));
	flash_float_to_byte(save_arry, (short)5, &(pit_d));

	flash_float_to_byte(save_arry, (short)6, &(yaw_p));
	flash_float_to_byte(save_arry, (short)7, &(yaw_i));
	flash_float_to_byte(save_arry, (short)8, &(yaw_d));

	flash_float_to_byte(save_arry, (short)9, &(alt_p));
	flash_float_to_byte(save_arry, (short)10, &(alt_i));
	flash_float_to_byte(save_arry, (short)11, &(alt_d));

	flash_float_to_byte(save_arry, (short)12, &(pos_p));
	flash_float_to_byte(save_arry, (short)13, &(pos_i));
	flash_float_to_byte(save_arry, (short)14, &(pos_d));

	flash_float_to_byte(save_arry, (short)15, &(pid6_p));
	flash_float_to_byte(save_arry, (short)16, &(pid6_i));
	flash_float_to_byte(save_arry, (short)17, &(pid6_d));

	flash_float_to_byte(save_arry, (short)18, &(pid7_p));
	flash_float_to_byte(save_arry, (short)19, &(pid7_i));
	flash_float_to_byte(save_arry, (short)20, &(pid7_d));

	flash_float_to_byte(save_arry, (short)21, &(pid8_p));
	flash_float_to_byte(save_arry, (short)22, &(pid8_i));
	flash_float_to_byte(save_arry, (short)23, &(pid8_d));

	flash_float_to_byte(save_arry, (short)24, &(pid9_p));
	flash_float_to_byte(save_arry, (short)25, &(pid9_i));
	flash_float_to_byte(save_arry, (short)26, &(pid9_d));

	flash_float_to_byte(save_arry, (short)27, &(pid10_p));
	flash_float_to_byte(save_arry, (short)28, &(pid10_i));
	flash_float_to_byte(save_arry, (short)29, &(pid10_d));

	flash_float_to_byte(save_arry, (short)30, &(pid11_p));
	flash_float_to_byte(save_arry, (short)31, &(pid11_i));
	flash_float_to_byte(save_arry, (short)32, &(pid11_d));

	flash_float_to_byte(save_arry, (short)33, &(pid12_p));
	flash_float_to_byte(save_arry, (short)34, &(pid12_i));
	flash_float_to_byte(save_arry, (short)35, &(pid12_d));

	flash_float_to_byte(save_arry, (short)36, &(pid13_p));
	flash_float_to_byte(save_arry, (short)37, &(pid13_i));
	flash_float_to_byte(save_arry, (short)38, &(pid13_d));

	flash_float_to_byte(save_arry, (short)39, &(pid14_p));
	flash_float_to_byte(save_arry, (short)40, &(pid14_i));
	flash_float_to_byte(save_arry, (short)41, &(pid14_d));

	flash_float_to_byte(save_arry, (short)42, &(pid15_p));
	flash_float_to_byte(save_arry, (short)43, &(pid15_i));
	flash_float_to_byte(save_arry, (short)44, &(pid15_d));

	flash_float_to_byte(save_arry, (short)45, &(pid16_p));
	flash_float_to_byte(save_arry, (short)46, &(pid16_i));
	flash_float_to_byte(save_arry, (short)47, &(pid16_d));

	flash_float_to_byte(save_arry, (short)48, &(pid17_p));
	flash_float_to_byte(save_arry, (short)49, &(pid17_i));
	flash_float_to_byte(save_arry, (short)50, &(pid17_d));

	flash_float_to_byte(save_arry, (short)51, &(pid18_p));
	flash_float_to_byte(save_arry, (short)52, &(pid18_i));
	flash_float_to_byte(save_arry, (short)53, &(pid18_d));
        
        flash_float_to_byte(save_arry, (short)54, &(gyrox_raw_bias_dps));
	flash_float_to_byte(save_arry, (short)55, &(gyroy_raw_bias_dps));
	flash_float_to_byte(save_arry, (short)56, &(gyroz_raw_bias_dps));
        
        flash_float_to_byte(save_arry, (short)57, &(accx_raw_bias_mps));
	flash_float_to_byte(save_arry, (short)58, &(accy_raw_bias_mps));
	flash_float_to_byte(save_arry, (short)59, &(accz_raw_bias_mps));

	UpdateTheFLASH(FLASH_USER_START_ADDR, (uint8_t *)save_arry, sizeof(save_arry));
}

uint8_t flash_read_parameters(void)
{
	ReadTheFLASH(FLASH_USER_START_ADDR, (uint8_t *)save_arry, sizeof(save_arry));
	if (save_arry[0] == 0xff &&
		save_arry[1] == 0xfe &&
		save_arry[max_flash_save_len - 2] == 0xfe &&
		save_arry[max_flash_save_len - 1] == 0xff)
	{
		flash_byte_to_float((uint8_t *)save_arry, (short)0, &(rol_p));
		flash_byte_to_float((uint8_t *)save_arry, (short)1, &(rol_i));
		flash_byte_to_float((uint8_t *)save_arry, (short)2, &(rol_d));

		flash_byte_to_float(save_arry, (short)3, &(pit_p));
		flash_byte_to_float(save_arry, (short)4, &(pit_i));
		flash_byte_to_float(save_arry, (short)5, &(pit_d));

		flash_byte_to_float(save_arry, (short)6, &(yaw_p));
		flash_byte_to_float(save_arry, (short)7, &(yaw_i));
		flash_byte_to_float(save_arry, (short)8, &(yaw_d));

		flash_byte_to_float(save_arry, (short)9, &(alt_p));
		flash_byte_to_float(save_arry, (short)10, &(alt_i));
		flash_byte_to_float(save_arry, (short)11, &(alt_d));

		flash_byte_to_float(save_arry, (short)12, &(pos_p));
		flash_byte_to_float(save_arry, (short)13, &(pos_i));
		flash_byte_to_float(save_arry, (short)14, &(pos_d));

		flash_byte_to_float(save_arry, (short)15, &(pid6_p));
		flash_byte_to_float(save_arry, (short)16, &(pid6_i));
		flash_byte_to_float(save_arry, (short)17, &(pid6_d));

		flash_byte_to_float(save_arry, (short)18, &(pid7_p));
		flash_byte_to_float(save_arry, (short)19, &(pid7_i));
		flash_byte_to_float(save_arry, (short)20, &(pid7_d));

		flash_byte_to_float(save_arry, (short)21, &(pid8_p));
		flash_byte_to_float(save_arry, (short)22, &(pid8_i));
		flash_byte_to_float(save_arry, (short)23, &(pid8_d));

		flash_byte_to_float(save_arry, (short)24, &(pid9_p));
		flash_byte_to_float(save_arry, (short)25, &(pid9_i));
		flash_byte_to_float(save_arry, (short)26, &(pid9_d));

		flash_byte_to_float(save_arry, (short)27, &(pid10_p));
		flash_byte_to_float(save_arry, (short)28, &(pid10_i));
		flash_byte_to_float(save_arry, (short)29, &(pid10_d));

		flash_byte_to_float(save_arry, (short)30, &(pid11_p));
		flash_byte_to_float(save_arry, (short)31, &(pid11_i));
		flash_byte_to_float(save_arry, (short)32, &(pid11_d));

		flash_byte_to_float(save_arry, (short)33, &(pid12_p));
		flash_byte_to_float(save_arry, (short)34, &(pid12_i));
		flash_byte_to_float(save_arry, (short)35, &(pid12_d));

		flash_byte_to_float(save_arry, (short)36, &(pid13_p));
		flash_byte_to_float(save_arry, (short)37, &(pid13_i));
		flash_byte_to_float(save_arry, (short)38, &(pid13_d));

		flash_byte_to_float(save_arry, (short)39, &(pid14_p));
		flash_byte_to_float(save_arry, (short)40, &(pid14_i));
		flash_byte_to_float(save_arry, (short)41, &(pid14_d));

		flash_byte_to_float(save_arry, (short)42, &(pid15_p));
		flash_byte_to_float(save_arry, (short)43, &(pid15_i));
		flash_byte_to_float(save_arry, (short)44, &(pid15_d));

		flash_byte_to_float(save_arry, (short)45, &(pid16_p));
		flash_byte_to_float(save_arry, (short)46, &(pid16_i));
		flash_byte_to_float(save_arry, (short)47, &(pid16_d));

		flash_byte_to_float(save_arry, (short)48, &(pid17_p));
		flash_byte_to_float(save_arry, (short)49, &(pid17_i));
		flash_byte_to_float(save_arry, (short)50, &(pid17_d));

		flash_byte_to_float(save_arry, (short)51, &(pid18_p));
		flash_byte_to_float(save_arry, (short)52, &(pid18_i));
		flash_byte_to_float(save_arry, (short)53, &(pid18_d));
                
                flash_byte_to_float(save_arry, (short)54, &(gyrox_raw_bias_dps));
                flash_byte_to_float(save_arry, (short)55, &(gyroy_raw_bias_dps));
                flash_byte_to_float(save_arry, (short)56, &(gyroz_raw_bias_dps));
                
                flash_byte_to_float(save_arry, (short)57, &(accx_raw_bias_mps));
                flash_byte_to_float(save_arry, (short)58, &(accy_raw_bias_mps));
                flash_byte_to_float(save_arry, (short)59, &(accz_raw_bias_mps));

		return 0;
	}
	else
	{
		return 1;
	}
}

static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
	{
		sector = FLASH_SECTOR_7;
	}
	return sector;
}

void FLASHinit(void)
{
	uint32_t FirstSector = 0, NbOfSectors = 0;

	FirstSector = GetSector(FLASH_USER_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
	//NbOfSectors=1;
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;
}

uint8_t UpdateTheFLASH(uint32_t Address, uint8_t *buf, short size)
{
	short number = 0;
	uint32_t  SECTORError;
	HAL_FLASH_Unlock();
	FLASHinit();
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		while (1)
		{
			//asm("nop");
			return 0;
		}
	}
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();
	// Address = FLASH_USER_START_ADDR;

	while (number<size)
	{
		number++;
		HAL_Delay(1);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, *buf) == HAL_OK)
		{
			Address = Address + 1;
			buf++;
		}
		else
		{
			/* Error occurred while writing data in Flash memory.
			User can add here some code to deal with this error */
			while (1)
			{
				asm("nop");
				return 0;
			}
		}
	}
	HAL_FLASH_Lock();
	return 1;
}

void ReadTheFLASH(uint32_t Address, uint8_t *buf, short size)
{
	short number = 0;
	while (number < size)
	{
		HAL_Delay(1);
		buf[number] = *(__IO uint8_t *)Address;
		Address = Address + 1;
		number++;
	}
}

void flash_byte_to_float(uint8_t * buf, short number, float * paramer)
{
	char byte_float[4];
	float paramer1;
	byte_float[0] = *(buf + 2 + number * 4);
	byte_float[1] = *(buf + 2 + number * 4 + 1);
	byte_float[2] = *(buf + 2 + number * 4 + 2);
	byte_float[3] = *(buf + 2 + number * 4 + 3);
	paramer1 = *((float*)(byte_float));
	*paramer = paramer1;
}

void flash_float_to_byte(uint8_t * buf, short number, float* paramer)
{
	unsigned char *point;
	point = (unsigned char *)paramer; //得到float的地址
	buf[2 + number * 4] = point[0];
	buf[2 + number * 4 + 1] = point[1];
	buf[2 + number * 4 + 2] = point[2];
	buf[2 + number * 4 + 3] = point[3];
}


