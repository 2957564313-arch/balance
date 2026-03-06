/*********************************************************************************************************************
* STC32G Opensourec Library 鍗筹紙STC32G 寮€婧愬簱锛夋槸涓€涓熀浜庡畼鏂?SDK 鎺ュ彛鐨勭涓夋柟寮€婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰绉戞妧
*
* 鏈枃浠舵槸STC 寮€婧愬簱鐨勪竴閮ㄥ垎
*
* STC32G 寮€婧愬簱 鏄厤璐硅蒋浠?
* 鎮ㄥ彲浠ユ牴鎹嚜鐢辫蒋浠跺熀閲戜細鍙戝竷鐨?GPL锛圙NU General Public License锛屽嵆 GNU閫氱敤鍏叡璁稿彲璇侊級鐨勬潯娆?
* 鍗?GPL 鐨勭3鐗堬紙鍗?GPL3.0锛夋垨锛堟偍閫夋嫨鐨勶級浠讳綍鍚庢潵鐨勭増鏈紝閲嶆柊鍙戝竷鍜?鎴栦慨鏀瑰畠
*
* 鏈紑婧愬簱鐨勫彂甯冩槸甯屾湜瀹冭兘鍙戞尌浣滅敤锛屼絾骞舵湭瀵瑰叾浣滀换浣曠殑淇濊瘉
* 鐢氳嚦娌℃湁闅愬惈鐨勯€傞攢鎬ф垨閫傚悎鐗瑰畾鐢ㄩ€旂殑淇濊瘉
* 鏇村缁嗚妭璇峰弬瑙?GPL
*
* 鎮ㄥ簲璇ュ湪鏀跺埌鏈紑婧愬簱鐨勫悓鏃舵敹鍒颁竴浠?GPL 鐨勫壇鏈?
* 濡傛灉娌℃湁锛岃鍙傞槄<https://www.gnu.org/licenses/>
*
* 棰濆娉ㄦ槑锛?
* 鏈紑婧愬簱浣跨敤 GPL3.0 寮€婧愯鍙瘉鍗忚 浠ヤ笂璁稿彲鐢虫槑涓鸿瘧鏂囩増鏈?
* 璁稿彲鐢虫槑鑻辨枃鐗堝湪 libraries/doc 鏂囦欢澶逛笅鐨?GPL3_permission_statement.txt 鏂囦欢涓?
* 璁稿彲璇佸壇鏈湪 libraries 鏂囦欢澶逛笅 鍗宠鏂囦欢澶逛笅鐨?LICENSE 鏂囦欢
* 娆㈣繋鍚勪綅浣跨敤骞朵紶鎾湰绋嬪簭 浣嗕慨鏀瑰唴瀹规椂蹇呴』淇濈暀閫愰绉戞妧鐨勭増鏉冨０鏄庯紙鍗虫湰澹版槑锛?
*
* 鏂囦欢鍚嶇О          
* 鍏徃鍚嶇О          鎴愰兘閫愰绉戞妧鏈夐檺鍏徃
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮€鍙戠幆澧?         MDK FOR C251
* 閫傜敤骞冲彴          STC32G
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
*
* 淇敼璁板綍
* 鏃ユ湡              浣滆€?          澶囨敞
* 2024-08-01        澶            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 鎺ョ嚎瀹氫箟:
*                   ------------------------------------
*                   妯″潡绠¤剼                                        鍗曠墖鏈虹鑴?
*                   //------------------纭欢 SPI 寮曡剼------------------//
*                   SCL/SPC             鏌ョ湅 zf_device_icm20602.h 涓?ICM20602_SPC_PIN 瀹忓畾涔?
*                   SDA/DSI             鏌ョ湅 zf_device_icm20602.h 涓?ICM20602_SDI_PIN 瀹忓畾涔?
*                   SA0/SDO             鏌ョ湅 zf_device_icm20602.h 涓?ICM20602_SDO_PIN 瀹忓畾涔?
*                   CS                  鏌ョ湅 zf_device_icm20602.h 涓?IPS114_CS_PIN 瀹忓畾涔?
*                   //------------------纭欢 SPI 寮曡剼------------------//
*                   //------------------杞欢 IIC 寮曡剼------------------//
*                   SCL/SPC             鏌ョ湅 zf_device_icm20602.h 涓?ICM20602_SCL_PIN 瀹忓畾涔?
*                   SDA/DSI             鏌ョ湅 zf_device_icm20602.h 涓?ICM20602_SDA_PIN 瀹忓畾涔?
*                   //------------------杞欢 IIC 寮曡剼------------------//
*                   鐢垫簮寮曡剼
*                   VCC                 3.3V鐢垫簮
*                   GND                 鐢垫簮鍦?
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_soft_spi.h"

#include "zf_device_icm20602.h"

#pragma warning disable = 183
#pragma warning disable = 177

int16 icm20602_gyro_x = 0, icm20602_gyro_y = 0, icm20602_gyro_z = 0;            // 涓夎酱闄€铻轰华鏁版嵁      gyro (闄€铻轰华)
int16 icm20602_acc_x = 0, icm20602_acc_y = 0, icm20602_acc_z = 0;               // 涓夎酱鍔犻€熷害璁℃暟鎹?   acc (accelerometer 鍔犻€熷害璁?
float icm20602_transition_factor[2] = {4096, 16.4};

#if (ICM20602_USE_INTERFACE==HARDWARE_SPI)                               // 杩欎袱娈?棰滆壊姝ｅ父鐨勬墠鏄纭殑 棰滆壊鐏扮殑灏辨槸娌℃湁鐢ㄧ殑

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    ICM20602 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     data            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_write_register (uint8 reg, uint8 dat)
	{
		ICM20602_CS(0);
		spi_write_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_W, dat);
		ICM20602_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    ICM20602 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8           鏁版嵁
	// 浣跨敤绀轰緥     icm20602_read_register(ICM20602_WHO_AM_I);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 icm20602_read_register (uint8 reg)
	{
		uint8 dat = 0;
		ICM20602_CS(0);
		dat = spi_read_8bit_register(ICM20602_SPI, reg | ICM20602_SPI_R);
		ICM20602_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    ICM20602 璇绘暟鎹?
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     data            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_read_registers (uint8 reg, uint8 *dat, uint32 len)
	{
		ICM20602_CS(0);
		spi_read_8bit_registers(ICM20602_SPI, reg | ICM20602_SPI_R, dat, len);
		ICM20602_CS(1);
	}

#elif (ICM20602_USE_INTERFACE==SOFT_SPI)
	#define ICM20602_SCK(x)				(ICM20602_SPC_PIN 	= x)
	#define ICM20602_MOSI(x) 			(ICM20602_SDI_PIN 	= x)
	#define ICM20602_MISO    			(ICM20602_SDO_PIN	   )
	#define ICM20602_CS(x)              (ICM20602_CS_PIN 	= x)
	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      閫氳繃SPI鍐欎竴涓猙yte,鍚屾椂璇诲彇涓€涓猙yte
	//  @param      byte        鍙戦€佺殑鏁版嵁
	//  @return     uint8 edata       return 杩斿洖status鐘舵€?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 icm20602_simspi_wr_byte(uint8 byte)
	{
		uint8 i;
		for(i=0; i<8; i++)
		{
			ICM20602_SCK (0);
			ICM20602_MOSI(byte&0x80);
			byte <<= 1;
			ICM20602_SCK (1);
			byte |= ICM20602_MISO;
		}
		return(byte);
	}
	
	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      灏唙al鍐欏叆cmd瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃,鍚屾椂杩斿洖status瀛楄妭
	//  @param      cmd         鍛戒护瀛?
	//  @param      val         寰呭啓鍏ュ瘎瀛樺櫒鐨勬暟鍊?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_simspi_w_reg_byte(uint8 cmd, uint8 val)
	{
		cmd |= ICM20602_SPI_W;
		icm20602_simspi_wr_byte(cmd);
		icm20602_simspi_wr_byte(val);
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      璇诲彇cmd鎵€瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃
	//  @param      cmd         鍛戒护瀛?
	//  @param      *val        瀛樺偍璇诲彇鐨勬暟鎹湴鍧€
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_simspi_r_reg_byte(uint8 cmd, uint8 *val)
	{
		cmd |= ICM20602_SPI_R;
		icm20602_simspi_wr_byte(cmd);
		*val = icm20602_simspi_wr_byte(0);
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      璇诲彇cmd鎵€瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃
	//  @param      cmd         鍛戒护瀛?
	//  @param      *val        瀛樺偍璇诲彇鐨勬暟鎹湴鍧€
	//  @param      num         璇诲彇鐨勬暟閲?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
	{
		uint32 edata i = 0;
		cmd |= ICM20602_SPI_R;
		icm20602_simspi_wr_byte(cmd);
		while(num--)
		{
			*val++ = icm20602_simspi_wr_byte(0);
		}
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     icm20602_write_register(ICM20602_PWR_CONF, 0x00);                   // 鍏抽棴楂樼骇鐪佺數妯″紡
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_write_register(uint8 reg, uint8 dat)
	{
		ICM20602_CS(0);
		icm20602_simspi_w_reg_byte(reg | ICM20602_SPI_W, dat);
		ICM20602_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8 edata           鏁版嵁
	// 浣跨敤绀轰緥     icm20602_read_register(ICM20602_CHIP_ID);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 icm20602_read_register(uint8 reg)
	{
		uint8 dat;
		ICM20602_CS(0);
		icm20602_simspi_r_reg_byte(reg | ICM20602_SPI_R, &dat);
		ICM20602_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU660RA 璇绘暟鎹?
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     icm20602_read_registers(ICM20602_ACC_ADDRESS, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void icm20602_read_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		ICM20602_CS(0);
		icm20602_simspi_r_reg_bytes(reg | ICM20602_SPI_R, dat, len);
		ICM20602_CS(1);
	}

#elif (ICM20602_USE_INTERFACE==SOFT_IIC)

	static soft_iic_info_struct icm20602_iic_struct;

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    ICM20602 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	#define icm20602_write_register(reg, dat)      (soft_iic_write_8bit_register(&icm20602_iic_struct, (reg), (dat)))

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    ICM20602 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8           鏁版嵁
	// 浣跨敤绀轰緥     icm20602_read_register(ICM20602_WHO_AM_I);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	#define icm20602_read_register(reg)             (soft_iic_read_8bit_register(&icm20602_iic_struct, (reg)))

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    ICM20602 璇绘暟鎹?
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	#define icm20602_read_registers(reg, dat, len) (soft_iic_read_8bit_registers(&icm20602_iic_struct, (reg), (dat), (len)))

#endif


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    ICM20602 鑷
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鑷澶辫触 0-鑷鎴愬姛
// 浣跨敤绀轰緥     icm20602_self_check();
// 澶囨敞淇℃伅     鍐呴儴璋冪敤
//-------------------------------------------------------------------------------------------------------------------
static uint8 icm20602_self_check (void)
{
    uint8 dat = 0, return_state = 0;
    uint16 timeout_count = 0;
    
    while(0x12 != dat)                                                          // 鍒ゆ柇 ID 鏄惁姝ｇ‘
    {
        if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state =  1;
            break;
        }
        
        dat = icm20602_read_register(ICM20602_WHO_AM_I);
        system_delay_ms(10);
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇 ICM20602 鍔犻€熷害璁℃暟鎹?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     icm20602_get_acc();                                             // 鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_acc (void)
{
    uint8 dat[6];
    
    icm20602_read_registers(ICM20602_ACCEL_XOUT_H, dat, 6);
    icm20602_acc_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_acc_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_acc_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇ICM20602闄€铻轰华鏁版嵁
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     icm20602_get_gyro();                                            // 鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
void icm20602_get_gyro (void)
{
    uint8 dat[6];
    
    icm20602_read_registers(ICM20602_GYRO_XOUT_H, dat, 6);
    icm20602_gyro_x = (int16)(((uint16)dat[0] << 8 | dat[1]));
    icm20602_gyro_y = (int16)(((uint16)dat[2] << 8 | dat[3]));
    icm20602_gyro_z = (int16)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鍒濆鍖?ICM20602
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鍒濆鍖栧け璐?0-鍒濆鍖栨垚鍔?
// 浣跨敤绀轰緥     icm20602_init();
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
uint8 icm20602_init (void)
{
    uint8 val = 0x0, return_state = 0;
    uint16 timeout_count = 0;
    
    system_delay_ms(10);                                                        // 涓婄數寤舵椂
    
#if (ICM20602_USE_INTERFACE==HARDWARE_SPI)
	
	spi_init(ICM20602_SPI, SPI_MODE0, ICM20602_SPI_SPEED, ICM20602_SPC_PIN, ICM20602_SDI_PIN, ICM20602_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM20602_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	
#elif (ICM20602_USE_INTERFACE==SOFT_SPI)
	
	// 榛樿浣跨敤鍙屽悜IO锛屼笉闇€瑕佸垵濮嬪寲銆?
//	soft_spi_init(ICM20602_SPI, SPI_MODE0, 0, ICM20602_SPC_PIN, ICM20602_SDI_PIN, ICM20602_SDO_PIN, ICM20602_CS_PIN);

#elif (ICM20602_USE_INTERFACE==SOFT_IIC)
	
	soft_iic_init(&icm20602_iic_struct, ICM20602_DEV_ADDR, ICM20602_SOFT_IIC_DELAY, ICM20602_SCL_PIN, ICM20602_SDA_PIN);

#endif
	
    
    do
    {
        if(icm20602_self_check())
        {
            // 濡傛灉绋嬪簭鍦ㄨ緭鍑轰簡鏂█淇℃伅 骞朵笖鎻愮ず鍑洪敊浣嶇疆鍦ㄨ繖閲?
            // 閭ｄ箞灏辨槸 ICM20602 鑷鍑洪敊骞惰秴鏃堕€€鍑轰簡
            // 妫€鏌ヤ竴涓嬫帴绾挎湁娌℃湁闂 濡傛灉娌￠棶棰樺彲鑳藉氨鏄潖浜?
            // printf( "icm20602 self check error.\r\n");
            return_state = 1;
            break;
        }
        
        icm20602_write_register(ICM20602_PWR_MGMT_1, 0x80);                     // 澶嶄綅璁惧
        system_delay_ms(2);
        
        do
        {
            // 绛夊緟澶嶄綅鎴愬姛
            val = icm20602_read_register(ICM20602_PWR_MGMT_1);
            
            if(ICM20602_TIMEOUT_COUNT < timeout_count ++)
            {
                // 濡傛灉绋嬪簭鍦ㄨ緭鍑轰簡鏂█淇℃伅 骞朵笖鎻愮ず鍑洪敊浣嶇疆鍦ㄨ繖閲?
                // 閭ｄ箞灏辨槸 ICM20602 鑷鍑洪敊骞惰秴鏃堕€€鍑轰簡
                // 妫€鏌ヤ竴涓嬫帴绾挎湁娌℃湁闂 濡傛灉娌￠棶棰樺彲鑳藉氨鏄潖浜?
                // printf( "icm20602 reset error.\r\n");
                return_state = 1;
                break;
            }
        }
        while(0x41 != val);
        
        if(1 == return_state)
        {
            break;
        }
        
        icm20602_write_register(ICM20602_PWR_MGMT_1,     0x01);                 // 鏃堕挓璁剧疆
        icm20602_write_register(ICM20602_PWR_MGMT_2,     0x00);                 // 寮€鍚檧铻轰华鍜屽姞閫熷害璁?
        icm20602_write_register(ICM20602_CONFIG,         0x01);                 // 176HZ 1KHZ
        icm20602_write_register(ICM20602_SMPLRT_DIV,     0x07);                 // 閲囨牱閫熺巼 SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
        
        // ICM20602_ACCEL_CONFIG 瀵勫瓨鍣?
        // 璁剧疆涓?0x00 鍔犻€熷害璁￠噺绋嬩负 卤2  g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?16384  鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x08 鍔犻€熷害璁￠噺绋嬩负 卤4  g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?8192   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x10 鍔犻€熷害璁￠噺绋嬩负 卤8  g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?4096   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x18 鍔犻€熷害璁￠噺绋嬩负 卤16 g   鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?2048   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        switch(ICM20602_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                // printf( "ICM20602_ACC_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_2G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x00);
                icm20602_transition_factor[0] = 16384;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_4G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x08);
                icm20602_transition_factor[0] = 8192;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_8G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x10);
                icm20602_transition_factor[0] = 4096;
            }
            break;
            
            case ICM20602_ACC_SAMPLE_SGN_16G:
            {
                icm20602_write_register(ICM20602_ACCEL_CONFIG, 0x18);
                icm20602_transition_factor[0] = 2048;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        // ICM20602_GYRO_CONFIG 瀵勫瓨鍣?
        // 璁剧疆涓?0x00 闄€铻轰华閲忕▼涓?卤250  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 131     鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x08 闄€铻轰华閲忕▼涓?卤500  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 65.5    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x10 闄€铻轰华閲忕▼涓?卤1000 dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 32.8    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x18 闄€铻轰华閲忕▼涓?卤2000 dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 16.4    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        switch(ICM20602_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                // printf( "ICM20602_GYRO_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_250DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x00);
                icm20602_transition_factor[1] = 131.0;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_500DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x08);
                icm20602_transition_factor[1] = 65.5;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_1000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x10);
                icm20602_transition_factor[1] = 32.8;
            }
            break;
            
            case ICM20602_GYRO_SAMPLE_SGN_2000DPS:
            {
                icm20602_write_register(ICM20602_GYRO_CONFIG, 0x18);
                icm20602_transition_factor[1] = 16.4;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        icm20602_write_register(ICM20602_ACCEL_CONFIG_2, 0x03);                 // Average 4 samples   44.8HZ   //0x23 Average 16 samples
    }
    while(0);
    
    return return_state;
}
