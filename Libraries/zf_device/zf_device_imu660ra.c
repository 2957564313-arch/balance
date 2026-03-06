#include "zf_common_function.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_soft_spi.h"
#include "zf_device_config.h"

#include "zf_device_imu660ra.h"

#pragma warning disable = 183
#pragma warning disable = 177

int16 imu660ra_gyro_x = 0, imu660ra_gyro_y = 0, imu660ra_gyro_z = 0;            // 涓夎酱闄€铻轰华鏁版嵁   gyro (闄€铻轰华)
int16 imu660ra_acc_x = 0, imu660ra_acc_y = 0, imu660ra_acc_z = 0;               // 涓夎酱鍔犻€熷害璁℃暟鎹?acc  (accelerometer 鍔犻€熷害璁?
float imu660ra_transition_factor[2] = {8192, 32.8};                             // 杞崲绯绘暟锛岄粯璁ゅ€煎搴?卤4g 鍜?卤1000dps 閲忕▼锛岃绠楁柟娉曚负 32768 / 閲忕▼鍊硷紝鍗曚綅鍒嗗埆涓?LSB/g 鍜?LSB/(掳/s)


#if (IMU660RA_USE_INTERFACE==HARDWARE_SPI) 
	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                   // 鍏抽棴楂樼骇鐪佺數妯″紡
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_write_register(uint8 reg, uint8 dat)
	{
		IMU660RA_CS(0);
		spi_write_8bit_register(IMU660RA_SPI, reg | IMU660RA_SPI_W, dat);
		IMU660RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 鍐欐暟鎹?
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu660ra_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_write_registers(uint8 reg, const uint8 *dat, uint32 len)
	{
		IMU660RA_CS(0);
		spi_write_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_W, dat, len);
		IMU660RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8           鏁版嵁
	// 浣跨敤绀轰緥     imu660ra_read_register(IMU660RA_CHIP_ID);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu660ra_read_register(uint8 reg)
	{
		uint8 dat[2];
		IMU660RA_CS(0);
		spi_read_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_R, dat, 2);
		IMU660RA_CS(1);
		return dat[1];
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 璇绘暟鎹?
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_read_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		uint16 i = 0;
		uint8 temp_data[13];
		IMU660RA_CS(0);
		spi_read_8bit_registers(IMU660RA_SPI, reg | IMU660RA_SPI_R, temp_data, len + 1);
		IMU660RA_CS(1);
		
		for(i = 0; i < len; i ++)
		{
			*(dat ++) = temp_data[i + 1];
		}
	}


#elif (IMU660RA_USE_INTERFACE==SOFT_SPI)

	#define IMU660RA_SCK(x)				IMU660RA_SPC_PIN  = x
	#define IMU660RA_MOSI(x) 			IMU660RA_SDI_PIN = x
	#define IMU660RA_MISO    			IMU660RA_SDO_PIN
	#define IMU660RA_CS(x)  			IMU660RA_CS_PIN  = x

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      閫氳繃SPI鍐欎竴涓猙yte,鍚屾椂璇诲彇涓€涓猙yte
	//  @param      byte        鍙戦€佺殑鏁版嵁
	//  @return     uint8 edata       return 杩斿洖status鐘舵€?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu660ra_simspi_wr_byte(uint8 byte)
	{
		uint8 i;
		for(i=0; i<8; i++)
		{
			IMU660RA_SCK (0);
			IMU660RA_MOSI(byte&0x80);
			byte <<= 1;
			IMU660RA_SCK (1);
			byte |= IMU660RA_MISO;
		}
		IMU660RA_SCK (0);
		return(byte);
	}
	
	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      灏唙al鍐欏叆cmd瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃,鍚屾椂杩斿洖status瀛楄妭
	//  @param      cmd         鍛戒护瀛?
	//  @param      val         寰呭啓鍏ュ瘎瀛樺櫒鐨勬暟鍊?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_simspi_w_reg_byte(uint8 cmd, uint8 val)
	{
		cmd |= IMU660RA_SPI_W;
		imu660ra_simspi_wr_byte(cmd);
		imu660ra_simspi_wr_byte(val);
	}


	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      灏唙al鍐欏叆cmd瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃
	//  @param      cmd         鍛戒护瀛?
	//  @param      val         寰呭啓鍏ュ瘎瀛樺櫒鐨勬暟鍊?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_simspi_w_reg_bytes(uint8 cmd, uint8 *dat_addr, uint32 len)
	{
		cmd |= IMU660RA_SPI_W;
		imu660ra_simspi_wr_byte(cmd);
		while(len--)
		{
			imu660ra_simspi_wr_byte(*dat_addr++);
		}
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      璇诲彇cmd鎵€瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃
	//  @param      cmd         鍛戒护瀛?
	//  @param      *val        瀛樺偍璇诲彇鐨勬暟鎹湴鍧€
	//  @param      num         璇诲彇鐨勬暟閲?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
	{
		cmd |= IMU660RA_SPI_R;
		imu660ra_simspi_wr_byte(cmd);
		while(num--)
		{
			*val++ = imu660ra_simspi_wr_byte(0);
		}
	}


	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                   // 鍏抽棴楂樼骇鐪佺數妯″紡
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_write_register(uint8 reg, uint8 dat)
	{
		IMU660RA_CS(0);
		imu660ra_simspi_w_reg_byte(reg | IMU660RA_SPI_W, dat);
		IMU660RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 鍐欐暟鎹?
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu660ra_write_registers(IMU660RA_INIT_dat, imu660ra_config_file, sizeof(imu660ra_config_file));
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_write_registers(uint8 reg, const uint8 *dat, uint32 len)
	{
		IMU660RA_CS(0);
		imu660ra_simspi_w_reg_bytes(reg | IMU660RA_SPI_W, dat, len);
		IMU660RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8 edata           鏁版嵁
	// 浣跨敤绀轰緥     imu660ra_read_register(IMU660RA_CHIP_ID);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu660ra_read_register(uint8 reg)
	{
		uint8 dat[2];
		IMU660RA_CS(0);
		imu660ra_simspi_r_reg_bytes(reg | IMU660RA_SPI_R, dat, 2);
		IMU660RA_CS(1);
		return dat[1];
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 璇绘暟鎹?
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu660ra_read_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		uint16 i = 0;
		uint8 temp_data[13];
		IMU660RA_CS(0);
		imu660ra_simspi_r_reg_bytes(reg | IMU660RA_SPI_R, temp_data, len + 1);
		IMU660RA_CS(1);
		
		for(i = 0; i < len; i ++)
		{
			*(dat ++) = temp_data[i + 1];
		}
	}
	
#elif (IMU660RA_USE_INTERFACE==SOFT_IIC)

	static soft_iic_info_struct imu660ra_iic_struct;
	#define imu660ra_write_register(reg, dat)        (soft_iic_write_8bit_register(&imu660ra_iic_struct, (reg), (dat)))
	#define imu660ra_write_registers(reg, dat, len)  (soft_iic_write_8bit_registers(&imu660ra_iic_struct, (reg), (dat), (len)))
	#define imu660ra_read_register(reg)              (soft_iic_read_8bit_register(&imu660ra_iic_struct, (reg)))
	#define imu660ra_read_registers(reg, dat, len)   (soft_iic_read_8bit_registers(&imu660ra_iic_struct, (reg), (dat), (len)))

#endif

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    IMU660RA 鑷
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鑷澶辫触 0-鑷鎴愬姛
// 浣跨敤绀轰緥     imu660ra_self_check();
// 澶囨敞淇℃伅     鍐呴儴璋冪敤
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu660ra_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    
    do
    {
        if(timeout_count ++ > IMU660RA_TIMEOUT_COUNT)
        {
            return_state =  1;
            break;
        }
        
        dat = imu660ra_read_register(IMU660RA_CHIP_ID);
        // printf("imu660ra_read_register = 0x%X\r\n", dat);
        system_delay_ms(1);
    }
    while(0x24 != dat);                                                     // 璇诲彇璁惧ID鏄惁绛変簬0X24锛屽鏋滀笉鏄?X24鍒欒涓烘病妫€娴嬪埌璁惧
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇 IMU660RA 鍔犻€熷害璁℃暟鎹?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     imu660ra_get_acc();                                             // 鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
// 澶囨敞淇℃伅     浣跨敤 SPI 鐨勯噰闆嗘椂闂翠负69us
//            浣跨敤 IIC 鐨勯噰闆嗘椂闂翠负126us        閲囬泦鍔犻€熷害璁＄殑鏃堕棿涓庨噰闆嗛檧铻轰华鐨勬椂闂翠竴鑷寸殑鍘熷洜鏄兘鍙槸璇诲彇瀵勫瓨鍣ㄦ暟鎹?
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_get_acc (void)
{
    uint8 dat[6];
    
    imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 6);
    imu660ra_acc_x = (int16)(((uint16)dat[1] << 8 | dat[0]));
    imu660ra_acc_y = (int16)(((uint16)dat[3] << 8 | dat[2]));
    imu660ra_acc_z = (int16)(((uint16)dat[5] << 8 | dat[4]));
}
//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇 IMU660RA 闄€铻轰华鏁版嵁
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     imu660ra_get_gyro();                                            // 鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
// 澶囨敞淇℃伅     浣跨敤 SPI 鐨勯噰闆嗘椂闂翠负69us
//            浣跨敤 IIC 鐨勯噰闆嗘椂闂翠负126us
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_get_gyro (void)
{
    uint8 dat[6];
    
    imu660ra_read_registers(IMU660RA_GYRO_ADDRESS, dat, 6);
    imu660ra_gyro_x = (int16)(((uint16)dat[1] << 8 | dat[0]));
    imu660ra_gyro_y = (int16)(((uint16)dat[3] << 8 | dat[2]));
    imu660ra_gyro_z = (int16)(((uint16)dat[5] << 8 | dat[4]));
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇 IMU660RA 鍔犻€熷害璁″拰闄€铻轰华鏁版嵁
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     imu660ra_get_acc_gyro();                                       // 鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
// 澶囨敞淇℃伅     浣跨敤 SPI 鐨勯噰闆嗘椂闂翠负110us
//            浣跨敤 IIC 鐨勯噰闆嗘椂闂翠负200us
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_get_acc_gyro (void)
{
    uint8 dat[12];

    imu660ra_read_registers(IMU660RA_ACC_ADDRESS, dat, 12);

    imu660ra_acc_x  = (int16)(((uint16)dat[1]  << 8) | dat[0]);
    imu660ra_acc_y  = (int16)(((uint16)dat[3]  << 8) | dat[2]);
    imu660ra_acc_z  = (int16)(((uint16)dat[5]  << 8) | dat[4]);

    imu660ra_gyro_x = (int16)(((uint16)dat[7]  << 8) | dat[6]);
    imu660ra_gyro_y = (int16)(((uint16)dat[9]  << 8) | dat[8]);
    imu660ra_gyro_z = (int16)(((uint16)dat[11] << 8) | dat[10]);
}

void imu660ra_correct_gyro_offset(int16 gyro_x_offset, int16 gyro_y_offset, int16 gyro_z_offset, int16 deadzone)
{
	imu660ra_gyro_x -= gyro_x_offset;                          // 鍘婚櫎X杞撮浂鐐瑰亸宸?
	imu660ra_gyro_y -= gyro_y_offset;                          // 鍘婚櫎Y杞撮浂鐐瑰亸宸?
	imu660ra_gyro_z -= gyro_z_offset;                          // 鍘婚櫎Z杞撮浂鐐瑰亸宸?

	// 姝诲尯澶勭悊锛氱粷瀵瑰€煎皬浜庢鍖洪槇鍊肩殑鏁版嵁瑙嗕负闆?
	if(imu660ra_gyro_x > -deadzone && imu660ra_gyro_x < deadzone) imu660ra_gyro_x = 0;
	if(imu660ra_gyro_y > -deadzone && imu660ra_gyro_y < deadzone) imu660ra_gyro_y = 0;
	if(imu660ra_gyro_z > -deadzone && imu660ra_gyro_z < deadzone) imu660ra_gyro_z = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鍒濆鍖?IMU660RA
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鍒濆鍖栧け璐?0-鍒濆鍖栨垚鍔?
// 浣跨敤绀轰緥     imu660ra_init();
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
uint8 imu660ra_init (void)
{
    uint8 return_state = 0;
    system_delay_ms(20);                                                        // 绛夊緟璁惧涓婄數鎴愬姛
    
	
#if (IMU660RA_USE_INTERFACE==HARDWARE_SPI)   
	spi_init(IMU660RA_SPI, SPI_MODE0, IMU660RA_SPI_SPEED, IMU660RA_SPC_PIN, IMU660RA_SDI_PIN, IMU660RA_SDO_PIN, SPI_CS_NULL);   // 閰嶇疆 IMU660RA 鐨?SPI 绔彛
    gpio_init(IMU660RA_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);         	 	// 閰嶇疆 IMU660RA 鐨凜S绔彛
	imu660ra_read_register(IMU660RA_CHIP_ID);                                   // 璇诲彇涓€涓嬭澶嘔D 灏嗚澶囪缃负SPI妯″紡
#elif (IMU660RA_USE_INTERFACE==SOFT_SPI)
	// 榛樿浣跨敤鍙屽悜IO锛屼笉闇€瑕佸垵濮嬪寲銆?
	// soft_spi_init (IMU660RA_SPI, SPI_MODE0, 0, IMU660RA_SPC_PIN, IMU660RA_SDI_PIN, IMU660RA_SDO_PIN, IMU660RA_CS_PIN);
    imu660ra_read_register(IMU660RA_CHIP_ID);          						// 璇诲彇涓€涓嬭澶嘔D 灏嗚澶囪缃负SPI妯″紡
#elif (IMU660RA_USE_INTERFACE==SOFT_IIC)
	soft_iic_init(&imu660ra_iic_struct, IMU660RA_DEV_ADDR, IMU660RA_SOFT_IIC_DELAY, IMU660RA_SCL_PIN, IMU660RA_SDA_PIN);        // 閰嶇疆 IMU660RA 鐨?IIC 绔彛
#endif

    
    do
    {
        if(imu660ra_self_check())                                               // IMU660RA 鑷
        {
            // 濡傛灉绋嬪簭鍦ㄨ緭鍑轰簡鏂█淇℃伅 骞朵笖鎻愮ず鍑洪敊浣嶇疆鍦ㄨ繖閲?
            // 閭ｄ箞灏辨槸 IMU660RA 鑷鍑洪敊骞惰秴鏃堕€€鍑轰簡
            // 妫€鏌ヤ竴涓嬫帴绾挎湁娌℃湁闂 濡傛灉娌￠棶棰樺彲鑳藉氨鏄潖浜?
            // printf( "imu660ra self check error.\r\n");
            return_state = 1;
            break;
        }
        
        imu660ra_write_register(IMU660RA_PWR_CONF, 0x00);                       // 鍏抽棴楂樼骇鐪佺數妯″紡
        system_delay_ms(1);
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x00);                      // 寮€濮嬪妯″潡杩涜鍒濆鍖栭厤缃?
        imu660ra_write_registers(IMU660RA_INIT_DATA, imu660ra_config_file, sizeof(imu660ra_config_file));   // 杈撳嚭閰嶇疆鏂囦欢
        imu660ra_write_register(IMU660RA_INIT_CTRL, 0x01);                      // 鍒濆鍖栭厤缃粨鏉?
        system_delay_ms(20);
        
        if(1 != imu660ra_read_register(IMU660RA_INT_STA))                       // 妫€鏌ユ槸鍚﹂厤缃畬鎴?
        {
            // 濡傛灉绋嬪簭鍦ㄨ緭鍑轰簡鏂█淇℃伅 骞朵笖鎻愮ず鍑洪敊浣嶇疆鍦ㄨ繖閲?
            // 閭ｄ箞灏辨槸 IMU660RA 閰嶇疆鍒濆鍖栨枃浠跺嚭閿欎簡
            // 妫€鏌ヤ竴涓嬫帴绾挎湁娌℃湁闂 濡傛灉娌￠棶棰樺彲鑳藉氨鏄潖浜?
            // printf( "imu660ra init error.\r\n");
            return_state = 1;
            break;
        }
        
        imu660ra_write_register(IMU660RA_PWR_CTRL, 0x0E);                       // 寮€鍚€ц兘妯″紡  浣胯兘闄€铻轰华銆佸姞閫熷害璁″拰娓╁害璁＄殑姝ｅ父閲囬泦
        imu660ra_write_register(IMU660RA_ACC_CONF, 0xA9);                       // 鍔犻€熷害閲囬泦閰嶇疆  姝ｅ父閲囬泦 5ms 閲囨牱棰戠巼
        imu660ra_write_register(IMU660RA_GYR_CONF, 0xEC);                       // 闄€铻轰华閲囬泦閰嶇疆  姝ｅ父閲囬泦 0.625ms 閲囨牱棰戠巼
        
        // IMU660RA_ACC_SAMPLE 瀵勫瓨鍣?
        // 璁剧疆涓?0x00 鍔犻€熷害璁￠噺绋嬩负 卤2  g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?16384  鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x01 鍔犻€熷害璁￠噺绋嬩负 卤4  g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?8192   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x02 鍔犻€熷害璁￠噺绋嬩负 卤8  g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?4096   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x03 鍔犻€熷害璁￠噺绋嬩负 卤16 g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?2048   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        switch(IMU660RA_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                // printf( "IMU660RA_ACC_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case IMU660RA_ACC_SAMPLE_SGN_2G:
            {
                imu660ra_write_register(IMU660RA_ACC_RANGE, 0x00);
                imu660ra_transition_factor[0] = 16384;
            }
            break;
            
            case IMU660RA_ACC_SAMPLE_SGN_4G:
            {
                imu660ra_write_register(IMU660RA_ACC_RANGE, 0x01);
                imu660ra_transition_factor[0] = 8192;
            }
            break;
            
            case IMU660RA_ACC_SAMPLE_SGN_8G:
            {
                imu660ra_write_register(IMU660RA_ACC_RANGE, 0x02);
                imu660ra_transition_factor[0] = 4096;
            }
            break;
            
            case IMU660RA_ACC_SAMPLE_SGN_16G:
            {
                imu660ra_write_register(IMU660RA_ACC_RANGE, 0x03);
                imu660ra_transition_factor[0] = 2048;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        // IMU660RA_GYR_RANGE 瀵勫瓨鍣?
        // 璁剧疆涓?0x04 闄€铻轰华閲忕▼涓?卤125  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 262.4   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x03 闄€铻轰华閲忕▼涓?卤250  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 131.2   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x02 闄€铻轰华閲忕▼涓?卤500  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 65.6    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x01 闄€铻轰华閲忕▼涓?卤1000 dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 32.8    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x00 闄€铻轰华閲忕▼涓?卤2000 dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 16.4    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        switch(IMU660RA_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                // printf( "IMU660RA_GYRO_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case IMU660RA_GYRO_SAMPLE_SGN_125DPS:
            {
                imu660ra_write_register(IMU660RA_GYR_RANGE, 0x04);
                imu660ra_transition_factor[1] = 262.4;
            }
            break;
            
            case IMU660RA_GYRO_SAMPLE_SGN_250DPS:
            {
                imu660ra_write_register(IMU660RA_GYR_RANGE, 0x03);
                imu660ra_transition_factor[1] = 131.2;
            }
            break;
            
            case IMU660RA_GYRO_SAMPLE_SGN_500DPS:
            {
                imu660ra_write_register(IMU660RA_GYR_RANGE, 0x02);
                imu660ra_transition_factor[1] = 65.6;
            }
            break;
            
            case IMU660RA_GYRO_SAMPLE_SGN_1000DPS:
            {
                imu660ra_write_register(IMU660RA_GYR_RANGE, 0x01);
                imu660ra_transition_factor[1] = 32.8;
            }
            break;
            
            case IMU660RA_GYRO_SAMPLE_SGN_2000DPS:
            {
                imu660ra_write_register(IMU660RA_GYR_RANGE, 0x00);
                imu660ra_transition_factor[1] = 16.4;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
    }
    while(0);
    
    return return_state;
}


