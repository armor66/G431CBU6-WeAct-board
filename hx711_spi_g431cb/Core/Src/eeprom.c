/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"
#include "bit_band.h"
#include "main.h"

/* Declarations and definitions ----------------------------------------------*/
//#define FLASH_KEY1               0x45670123
//#define FLASH_KEY2               0xCDEF89AB

#define SETTINGS_BYTES			16 //(8)half-words (% sizeof(uint64_t)) == 0)
#define SETTINGS_PAGE			63
//#define FLASH_PAGE_SIZE			0x00000800U
#define SETTINGS_PAGE_ADDR		0x0801F800UL
#define SETTINGS_FLAG			0xAA
#define SETTINGS_SIZE			8

int16_t settings_index = 0;
uint32_t saved_settings_address = 0;
uint8_t settings_array[SETTINGS_SIZE];

/* Functions -----------------------------------------------------------------*/
static void UTIL_MEM_cpy_8( void *dst, const void *src, uint16_t size )
{
  uint8_t* dst8= (uint8_t *) dst;
  uint8_t* src8= (uint8_t *) src;

  while( size-- )
    {
        *dst8++ = *src8++;
    }
}
static void flash_doubleWord(uint32_t Address, uint64_t Data)
{
  /* Set PG bit */
  SET_BIT(FLASH->CR, FLASH_CR_PG);
  /* Program first word */
  *(uint32_t *)Address = (uint32_t)Data;
  /* Barrier to ensure programming is performed in 2 steps, in right order (independently of compiler optimization behavior) */
  __ISB();
  /* Program second word */
  *(uint32_t *)(Address + 4U) = (uint32_t)(Data >> 32U);
}
static void flash_write64(uint32_t address, uint64_t data)
{
  while (*(uint64_t *)address != data)
  {
//	  while (FLASH->SR & FLASH_SR_PESD);	//((READ_BIT(FLASH->SR, FLASH_SR_PESD) == (FLASH_SR_PESD)) ? 1UL : 0UL)

//	  FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
	  while (FLASH->SR & FLASH_SR_BSY);
	  SET_BIT(FLASH->CR, FLASH_CR_PG);
	  flash_doubleWord(address, data);
//	  FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
	  while (FLASH->SR & FLASH_SR_BSY);
	  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
  }
}

void flash_lock(void)
{
	BIT_BAND_PERI(FLASH->CR, FLASH_CR_LOCK) = 1;	//lock flash
}
uint8_t flash_unlock(void)
{
	if(FLASH->CR & FLASH_CR_LOCK)
	  {
	    /* Authorize the FLASH Registers access */
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	    /* verify Flash is unlock */
		if(FLASH->CR & FLASH_CR_LOCK)
		{
			return 0;
		}
	  }
	return 1;
}

void flash_erase_page(uint32_t page)
{
//	  FLASH_EraseInitTypeDef erase_str;
//	  uint32_t page_error;
//	  erase_str.TypeErase = FLASH_TYPEERASE_PAGES;
//	  erase_str.Page = page;
//	  erase_str.NbPages = 1;
//    HAL_FLASHEx_Erase(&erase_str, &page_error);

	while (FLASH->SR & FLASH_SR_BSY);
//	while (FLASH->SR & FLASH_SR_PESD);	//When set, new program or erase operations are not started
//	BIT_BAND_PERI(FLASH->CR, FLASH_CR_EOPIE) = 1;
	MODIFY_REG(FLASH->CR, FLASH_CR_PNB, ((page << FLASH_CR_PNB_Pos) | FLASH_CR_PER | FLASH_CR_STRT));
	__NOP();	//HAL_Delay(10);
	BIT_BAND_PERI(FLASH->CR, FLASH_CR_STRT) = 1;
	while (FLASH->SR & FLASH_SR_BSY);
//		while (!(FLASH->SR & FLASH_SR_EOP));	//set by hardware if the end of operation interrupts are enabled(EOPIE = 1)
//		FLASH->SR = FLASH_SR_EOP;
	CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));	//BIT_BAND_PERI(FLASH->CR, FLASH_CR_PER) = 0;	//unset erase mode
}

void flash_write_array(uint32_t pDestination, uint8_t *pSource, uint32_t uLength)
{
	uint8_t *pSrc = pSource;
	uint64_t src_value;

	for (uint32_t i = 0; i < (uLength / sizeof(uint64_t)); i++)
	{
	    UTIL_MEM_cpy_8(&src_value, pSrc, sizeof(uint64_t));

	    /* Avoid writing 0xFFFFFFFFFFFFFFFFLL on erased Flash */
	    if (src_value != UINT64_MAX)
	    {
	      flash_write64(pDestination, src_value);
	    }

	    pDestination += sizeof(uint64_t);
	    pSrc += sizeof(uint64_t);
	}
}


void read_page(uint32_t start_address, uint8_t data_array[], uint16_t amount)
{
	for (uint8_t i = 0; i < amount; i++)
	{
		data_array[i] = ((__IO uint8_t *)start_address)[i];
	}
}

static uint8_t find_settings(void)
{

	uint8_t settings_found = 0;
	uint16_t end_index = (FLASH_PAGE_SIZE / SETTINGS_BYTES);
//search from top of the page
	saved_settings_address = SETTINGS_PAGE_ADDR + FLASH_PAGE_SIZE;	// - SAVED_GROUP_SIZE;	//max available

//from 256 to 1 (256 iterations)
    for(settings_index = end_index; settings_index > 0; settings_index--)
    {
    	saved_settings_address -= SETTINGS_BYTES;		//start from (top of the page - SAVED_GROUP_SIZE)

    	read_page(saved_settings_address, &settings_array[0], 1);

    	if (settings_array[0] == SETTINGS_FLAG)
        {
    		settings_found = 1;
            break;
        }
//        else saved_group_address[group] -= SAVED_GROUP_SIZE;
    }
//    settings_index -= 1;
    return settings_found;
}

uint32_t settings_load(void)
{
    if(find_settings())
    {
    	read_page(saved_settings_address, &settings_array[0], SETTINGS_SIZE);
    	settings_index--;
    }//else multiplier = 0;

    uint32_t value = (settings_array[4] << 24) + (settings_array[5] << 16) + (settings_array[6] << 8) + settings_array[7];

    return value;
}

void settings_save(uint32_t value)
{
	uint8_t settings_found = 0;
	uint8_t max_index = 0;
	uint32_t max_address = 0;

//find max index/address to save new settings in next free area
   	if(find_settings())
   	{
   		settings_found = 1;
       	if(settings_index > max_index)
       	{
       		max_index = settings_index;
       		max_address = saved_settings_address;
       	}
   	}//else settings_index = -1;

	//fill group_array[64]
   	settings_array[0] = SETTINGS_FLAG;	    //fill this after find procedure
   	settings_array[4] = (uint8_t)(value >> 24);
   	settings_array[5] = (uint8_t)(value >> 16);
   	settings_array[6] = (uint8_t)(value >> 8);
   	settings_array[7] = (uint8_t)value;

   	if(flash_unlock())	//write to next free area or erase entire page
   	{
   		if((max_index > (FLASH_PAGE_SIZE / SETTINGS_BYTES) - 1) || !settings_found)	//the page is full if last area occupied (index = 32)
   		{
   			flash_erase_page(SETTINGS_PAGE);
//todo: save and flash all saved groups from the beginning of SAVED_GROUPS_PAGE_ADDR
   			saved_settings_address = SETTINGS_PAGE_ADDR;			//start from first page address
   		}
   		else saved_settings_address = max_address + SETTINGS_BYTES;	//flash to next free area

   		settings_index = max_index;

   		flash_write_array(saved_settings_address, settings_array, SETTINGS_BYTES);
   		flash_lock();
   	}
}

