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
* 鎺ョ嚎瀹氫箟锛?
*                   ------------------------------------
*                   妯″潡绠¤剼            鍗曠墖鏈虹鑴?
*                   // 纭欢 SPI 寮曡剼
*                   SCL/SPC             鏌ョ湅 zf_device_imu963ra.h 涓?IMU963RA_SPC_PIN 瀹忓畾涔?
*                   SDA/DSI             鏌ョ湅 zf_device_imu963ra.h 涓?IMU963RA_SDI_PIN 瀹忓畾涔?
*                   SA0/SDO             鏌ョ湅 zf_device_imu963ra.h 涓?IMU963RA_SDO_PIN 瀹忓畾涔?
*                   CS                  鏌ョ湅 zf_device_imu963ra.h 涓?IMU963RA_CS_PIN  瀹忓畾涔?
*                   VCC                 3.3V鐢垫簮
*                   GND                 鐢垫簮鍦?
*                   鍏朵綑寮曡剼鎮┖
*
*                   // 杞欢 IIC 寮曡剼
*                   SCL/SPC             鏌ョ湅 zf_device_imu963ra.h 涓?IMU963RA_SCL_PIN 瀹忓畾涔?
*                   SDA/DSI             鏌ョ湅 zf_device_imu963ra.h 涓?IMU963RA_SDA_PIN 瀹忓畾涔?
*                   VCC                 3.3V鐢垫簮
*                   GND                 鐢垫簮鍦?
*                   鍏朵綑寮曡剼鎮┖
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_soft_spi.h"

#include "zf_device_imu963ra.h"

#pragma warning disable = 183
#pragma warning disable = 177

int16 imu963ra_gyro_x = 0, imu963ra_gyro_y = 0, imu963ra_gyro_z = 0;
int16 imu963ra_acc_x = 0,  imu963ra_acc_y = 0,  imu963ra_acc_z = 0;
int16 imu963ra_mag_x = 0,  imu963ra_mag_y = 0,  imu963ra_mag_z = 0;
float imu963ra_transition_factor[3] = {4098, 14.3, 3000};


#if (IMU963RA_USE_INTERFACE==HARDWARE_SPI)
	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu963ra_write_acc_gyro_register(uint8 reg, uint8 dat)
	{
		IMU963RA_CS(0);
		spi_write_8bit_register(IMU963RA_SPI, reg | IMU963RA_SPI_W, dat);
		IMU963RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8           鏁版嵁
	// 浣跨敤绀轰緥     imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu963ra_read_acc_gyro_register(uint8 reg)
	{
		uint8 dat = 0;
		IMU963RA_CS(0);
		dat = spi_read_8bit_register(IMU963RA_SPI, reg | IMU963RA_SPI_R);
		IMU963RA_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 璇绘暟鎹?鍐呴儴璋冪敤
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu963ra_read_acc_gyro_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		IMU963RA_CS(0);
		spi_read_8bit_registers(IMU963RA_SPI, reg | IMU963RA_SPI_R, dat, len);
		IMU963RA_CS(1);
	}
	
#elif (IMU963RA_USE_INTERFACE==SOFT_SPI)
	
	#define IMU963RA_SCK(x)				IMU963RA_SPC_PIN  = x
	#define IMU963RA_MOSI(x) 			IMU963RA_SDI_PIN = x
	#define IMU963RA_CS(x)  			IMU963RA_CS_PIN  = x
	#define IMU963RA_MISO    			IMU963RA_SDO_PIN

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      閫氳繃SPI鍐欎竴涓猙yte,鍚屾椂璇诲彇涓€涓猙yte
	//  @param      byte        鍙戦€佺殑鏁版嵁
	//  @return     uint8 edata       return 杩斿洖status鐘舵€?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu963ra_simspi_wr_byte(uint8 byte)
	{
		uint8 i;
		for(i=0; i<8; i++)
		{
			IMU963RA_SCK (0);
			IMU963RA_MOSI(byte&0x80);
			byte <<= 1;
			IMU963RA_SCK (1);
			byte |= IMU963RA_MISO;
		}
		IMU963RA_SCK (0);
		return(byte);
	}
	
	
	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      灏唙al鍐欏叆cmd瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃,鍚屾椂杩斿洖status瀛楄妭
	//  @param      cmd         鍛戒护瀛?
	//  @param      val         寰呭啓鍏ュ瘎瀛樺櫒鐨勬暟鍊?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu963ra_simspi_w_reg_byte(uint8 cmd, uint8 val)
	{
		IMU963RA_CS(0);
		cmd |= IMU963RA_SPI_W;
		imu963ra_simspi_wr_byte(cmd);
		imu963ra_simspi_wr_byte(val);
		IMU963RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      璇诲彇cmd鎵€瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃
	//  @param      cmd         鍛戒护瀛?
	//  @param      *val        瀛樺偍璇诲彇鐨勬暟鎹湴鍧€
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu963ra_simspi_r_reg_byte(uint8 cmd, uint8 *val)
	{
		IMU963RA_CS(0);
		cmd |= IMU963RA_SPI_R;
		imu963ra_simspi_wr_byte(cmd);
		*val = imu963ra_simspi_wr_byte(0);
		IMU963RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	//  @brief      璇诲彇cmd鎵€瀵瑰簲鐨勫瘎瀛樺櫒鍦板潃
	//  @param      cmd         鍛戒护瀛?
	//  @param      *val        瀛樺偍璇诲彇鐨勬暟鎹湴鍧€
	//  @param      num         璇诲彇鐨勬暟閲?
	//  @since      v1.0
	//  Sample usage:
	//-------------------------------------------------------------------------------------------------------------------
	static void imu963ra_simspi_r_reg_bytes(uint8 cmd, uint8 *val, uint32 num)
	{
		cmd |= IMU963RA_SPI_R;
		imu963ra_simspi_wr_byte(cmd);
		while(num--)
		{
			*val++ = imu963ra_simspi_wr_byte(0);
		}
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu963ra_write_acc_gyro_register(uint8 reg, uint8 dat)
	{
		IMU963RA_CS(0);
		imu963ra_simspi_w_reg_byte(reg | IMU963RA_SPI_W, dat);
		IMU963RA_CS(1);
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8 edata           鏁版嵁
	// 浣跨敤绀轰緥     imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static uint8 imu963ra_read_acc_gyro_register(uint8 reg)
	{
		uint8 dat = 0;
		IMU963RA_CS(0);
		imu963ra_simspi_r_reg_byte(reg | IMU963RA_SPI_R, &dat);
		IMU963RA_CS(1);
		return dat;
	}

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 璇绘暟鎹?鍐呴儴璋冪敤
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	static void imu963ra_read_acc_gyro_registers(uint8 reg, uint8 *dat, uint32 len)
	{
		IMU963RA_CS(0);
		imu963ra_simspi_r_reg_bytes( reg | IMU963RA_SPI_R, dat, len);
		IMU963RA_CS(1);
	}
	
#elif (IMU963RA_USE_INTERFACE==SOFT_IIC)
	static soft_iic_info_struct imu963ra_iic_struct;

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 鍐欏瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	#define imu963ra_write_acc_gyro_register(reg,dat)       (soft_iic_write_8bit_register(&imu963ra_iic_struct,reg,dat))

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 璇诲瘎瀛樺櫒
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 杩斿洖鍙傛暟     uint8           鏁版嵁
	// 浣跨敤绀轰緥     imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	#define imu963ra_read_acc_gyro_register(reg)             (soft_iic_sccb_read_register(&imu963ra_iic_struct,reg))

	//-------------------------------------------------------------------------------------------------------------------
	// 鍑芥暟绠€浠?    IMU963RA 璇绘暟鎹?鍐呴儴璋冪敤
	// 鍙傛暟璇存槑     reg             瀵勫瓨鍣ㄥ湴鍧€
	// 鍙傛暟璇存槑     dat            鏁版嵁缂撳啿鍖?
	// 鍙傛暟璇存槑     len             鏁版嵁闀垮害
	// 杩斿洖鍙傛暟     void
	// 浣跨敤绀轰緥     imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
	// 澶囨敞淇℃伅     鍐呴儴璋冪敤
	//-------------------------------------------------------------------------------------------------------------------
	#define imu963ra_read_acc_gyro_registers(reg,dat,len)   (soft_iic_read_8bit_registers(&imu963ra_iic_struct,reg,dat,len))
#endif

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    IMU963RA 浣滀负 IIC 涓绘満鍚戠鍔涜鍐欐暟鎹?
// 鍙傛暟璇存槑     addr            鐩爣鍦板潃
// 鍙傛暟璇存槑     reg             鐩爣瀵勫瓨鍣?
// 鍙傛暟璇存槑     dat            鏁版嵁
// 杩斿洖鍙傛暟     uint8           1-澶辫触 0-鎴愬姛
// 浣跨敤绀轰緥     imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x80);
// 澶囨敞淇℃伅     鍐呴儴璋冪敤
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_write_mag_register (uint8 addr, uint8 reg, uint8 dat)
{
    uint8 return_state = 0;
    uint16 timeout_count = 0;
    
    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);               // 浠庢満0閰嶇疆娓呴櫎
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 0);              // 璁剧疆鍦扮璁″湴鍧€锛堟敞鎰忚繖閲岄渶瑕佽缃?浣嶇殑I2C鍦板潃锛?0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // 闇€瑕佸啓鍏ョ殑瀵勫瓨鍣ㄥ湴鍧€
    imu963ra_write_acc_gyro_register(IMU963RA_DATAWRITE_SLV0, dat);            // 闇€瑕佸啓鍏ョ殑鏁版嵁
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);             // 浠呭湪绗竴涓懆鏈熷惎鐢ㄩ€氳 寮€鍚笂鎷?I2C涓绘満浣胯兘
    
    // 绛夊緟閫氳鎴愬姛
    while(0 == (0x80 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        
        system_delay_ms(2);
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    IMU963RA 浣滀负 IIC 涓绘満鍚戠鍔涜璇绘暟鎹?
// 鍙傛暟璇存槑     addr            鐩爣鍦板潃
// 鍙傛暟璇存槑     reg             鐩爣瀵勫瓨鍣?
// 杩斿洖鍙傛暟     uint8           璇诲彇鐨勬暟鎹?
// 浣跨敤绀轰緥     imu963ra_read_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CHIP_ID);
// 澶囨敞淇℃伅     鍐呴儴璋冪敤
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_read_mag_register (uint8 addr, uint8 reg)
{
    uint16 timeout_count = 0;
    
    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);              // 璁剧疆鍦扮璁″湴鍧€锛堟敞鎰忚繖閲岄渶瑕佽缃?浣嶇殑I2C鍦板潃锛?0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // 闇€瑕佽鍙栫殑瀵勫瓨鍣ㄥ湴鍧€
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x01);
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);             // 浠呭湪绗竴涓懆鏈熷惎鐢ㄩ€氳 寮€鍚笂鎷?I2C涓绘満浣胯兘
    
    // 绛夊緟閫氳鎴愬姛
    while(0 == (0x01 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            break;
        }
        
        system_delay_ms(2);
    }
    
    return (imu963ra_read_acc_gyro_register(IMU963RA_SENSOR_HUB_1));            // 杩斿洖璇诲彇鍒扮殑鏁版嵁
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    IMU963RA 浣滀负 IIC 涓绘満鍚戠鍔涜鑷姩鍐欐暟鎹?
// 鍙傛暟璇存槑     addr            鐩爣鍦板潃
// 鍙傛暟璇存槑     reg             鐩爣瀵勫瓨鍣?
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     imu963ra_connect_mag(IMU963RA_MAG_ADDR, IMU963RA_MAG_OUTX_L);
// 澶囨敞淇℃伅     鍐呴儴璋冪敤
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_connect_mag (uint8 addr, uint8 reg)
{
    addr = addr << 1;
    
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);              // 璁剧疆鍦扮璁″湴鍧€锛堟敞鎰忚繖閲岄渶瑕佽缃?浣嶇殑I2C鍦板潃锛?0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // 闇€瑕佽鍙栫殑瀵勫瓨鍣ㄥ湴鍧€
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x06);
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x6C);             // 浠呭湪绗竴涓懆鏈熷惎鐢ㄩ€氳 寮€鍚笂鎷?I2C涓绘満浣胯兘
}


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    IMU963RA 鍏酱鑷 鍐呴儴璋冪敤
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鑷澶辫触 0-鑷鎴愬姛
// 浣跨敤绀轰緥     imu963ra_acc_gyro_self_check();
// 澶囨敞淇℃伅     鍐呴儴璋冪敤
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_acc_gyro_self_check (void)
{
    uint8 return_state = 0;
    uint8 dat = 0;
    uint16 timeout_count = 0;
    
    while(0x6B != dat)                                                          // 鍒ゆ柇 ID 鏄惁姝ｇ‘
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        
        dat = imu963ra_read_acc_gyro_register(IMU963RA_WHO_AM_I);
		// printf("dat = %d\r\n", dat);
        system_delay_ms(10);
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    IMU963RA 纾佸姏璁¤嚜妫€ 鍐呴儴璋冪敤
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鑷澶辫触 0-鑷鎴愬姛
// 浣跨敤绀轰緥     imu963ra_mag_self_check();
// 澶囨敞淇℃伅     鍐呴儴璋冪敤
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu963ra_mag_self_check (void)
{
    uint8 return_state = 0;
    uint8 dat = 0;
    uint16 timeout_count = 0;
    
    while(0xff != dat)                                                          // 鍒ゆ柇 ID 鏄惁姝ｇ‘
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        
        dat = imu963ra_read_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CHIP_ID);
        system_delay_ms(10);
    }
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇 IMU963RA 鍔犻€熷害璁℃暟鎹?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     imu963ra_get_acc();
// 澶囨敞淇℃伅     鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_acc (void)
{
    uint8 dat[6];
    
    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
    imu963ra_acc_x = (int16)(((uint16)dat[1] << 8 | dat[0]));
    imu963ra_acc_y = (int16)(((uint16)dat[3] << 8 | dat[2]));
    imu963ra_acc_z = (int16)(((uint16)dat[5] << 8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇IMU963RA闄€铻轰华鏁版嵁
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     imu963ra_get_gyro();
// 澶囨敞淇℃伅     鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_gyro (void)
{
    uint8 dat[6];
    
    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_G, dat, 6);
    imu963ra_gyro_x = (int16)(((uint16)dat[1] << 8 | dat[0]));
    imu963ra_gyro_y = (int16)(((uint16)dat[3] << 8 | dat[2]));
    imu963ra_gyro_z = (int16)(((uint16)dat[5] << 8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇 IMU963RA 纾佸姏璁℃暟鎹?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     imu963ra_get_mag();
// 澶囨敞淇℃伅     鎵ц璇ュ嚱鏁板悗锛岀洿鎺ユ煡鐪嬪搴旂殑鍙橀噺鍗冲彲
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_mag (void)
{
    uint8 temp_status;
    uint8 dat[6];
    
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);
    temp_status = imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
    
    if(0x01 & temp_status)
    {
        imu963ra_read_acc_gyro_registers(IMU963RA_SENSOR_HUB_1, dat, 6);
        imu963ra_mag_x = (int16)(((uint16)dat[1] << 8 | dat[0]));
        imu963ra_mag_y = (int16)(((uint16)dat[3] << 8 | dat[2]));
        imu963ra_mag_z = (int16)(((uint16)dat[5] << 8 | dat[4]));
    }
    
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鍒濆鍖?IMU963RA
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鍒濆鍖栧け璐?0-鍒濆鍖栨垚鍔?
// 浣跨敤绀轰緥     imu963ra_init();
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
uint8 imu963ra_init (void)
{
    uint8 return_state = 0;
    system_delay_ms(10);                                                        // 涓婄數寤舵椂
    
#if (IMU963RA_USE_INTERFACE==HARDWARE_SPI)
	
	spi_init(IMU963RA_SPI, SPI_MODE0, IMU963RA_SPI_SPEED, IMU963RA_SPC_PIN, IMU963RA_SDI_PIN, IMU963RA_SDO_PIN, SPI_CS_NULL);
    gpio_init(IMU963RA_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	
#elif (IMU963RA_USE_INTERFACE==SOFT_SPI)
	
// 榛樿浣跨敤鍙屽悜IO锛屼笉闇€瑕佸垵濮嬪寲銆?
//	soft_spi_init(IMU963RA_SPI, SPI_MODE0, 0, IMU963RA_SPC_PIN, IMU963RA_SDI_PIN, IMU963RA_SDO_PIN, IMU963RA_CS_PIN);
	
#elif (IMU963RA_USE_INTERFACE==SOFT_IIC)
	
	soft_iic_init(&imu963ra_iic_struct, IMU963RA_DEV_ADDR, IMU963RA_SOFT_IIC_DELAY, IMU963RA_SCL_PIN, IMU963RA_SDA_PIN);
	
#endif
	
    do
    {
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // 鍏抽棴HUB瀵勫瓨鍣ㄨ闂?
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x01);               // 澶嶄綅璁惧
        system_delay_ms(2);
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // 鍏抽棴HUB瀵勫瓨鍣ㄨ闂?
        
        if(imu963ra_acc_gyro_self_check())
        {
            // printf( "IMU963RA acc and gyro self check error.");
            return_state = 1;
            break;
        }
        
        imu963ra_write_acc_gyro_register(IMU963RA_INT1_CTRL, 0x03);             // 寮€鍚檧铻轰华 鍔犻€熷害鏁版嵁灏辩华涓柇
        
        // IMU963RA_CTRL1_XL 瀵勫瓨鍣?
        // 璁剧疆涓?0x30 鍔犻€熷害閲忕▼涓?卤2  G    鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?16393  鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x38 鍔犻€熷害閲忕▼涓?卤4  G    鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?8197   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x3C 鍔犻€熷害閲忕▼涓?卤8  G    鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?4098   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        // 璁剧疆涓?0x34 鍔犻€熷害閲忕▼涓?卤16 G    鑾峰彇鍒扮殑鍔犻€熷害璁℃暟鎹櫎浠?2049   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 g(m/s^2)
        switch(IMU963RA_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                // printf( "IMU963RA_ACC_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case IMU963RA_ACC_SAMPLE_SGN_2G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x30);
                imu963ra_transition_factor[0] = 16393;
            }
            break;
            
            case IMU963RA_ACC_SAMPLE_SGN_4G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x38);
                imu963ra_transition_factor[0] = 8197;
            }
            break;
            
            case IMU963RA_ACC_SAMPLE_SGN_8G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x3C);
                imu963ra_transition_factor[0] = 4098;
            }
            break;
            
            case IMU963RA_ACC_SAMPLE_SGN_16G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x34);
                imu963ra_transition_factor[0] = 2049;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        // IMU963RA_CTRL2_G 瀵勫瓨鍣?
        // 璁剧疆涓?0x52 闄€铻轰华閲忕▼涓?卤125  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 228.6   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x50 闄€铻轰华閲忕▼涓?卤250  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 114.3   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x54 闄€铻轰华閲忕▼涓?卤500  dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 57.1    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x58 闄€铻轰华閲忕▼涓?卤1000 dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 28.6    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x5C 闄€铻轰华閲忕▼涓?卤2000 dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 14.3    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        // 璁剧疆涓?0x51 闄€铻轰华閲忕▼涓?卤4000 dps    鑾峰彇鍒扮殑闄€铻轰华鏁版嵁闄や互 7.1     鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅涓?掳/s
        switch(IMU963RA_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                // printf( "IMU963RA_GYRO_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case IMU963RA_GYRO_SAMPLE_SGN_125DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x52);
                imu963ra_transition_factor[1] = 228.6;
            }
            break;
            
            case IMU963RA_GYRO_SAMPLE_SGN_250DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x50);
                imu963ra_transition_factor[1] = 114.3;
            }
            break;
            
            case IMU963RA_GYRO_SAMPLE_SGN_500DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x54);
                imu963ra_transition_factor[1] = 57.1;
            }
            break;
            
            case IMU963RA_GYRO_SAMPLE_SGN_1000DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x58);
                imu963ra_transition_factor[1] = 28.6;
            }
            break;
            
            case IMU963RA_GYRO_SAMPLE_SGN_2000DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x5C);
                imu963ra_transition_factor[1] = 14.3;
            }
            break;
            
            case IMU963RA_GYRO_SAMPLE_SGN_4000DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x51);
                imu963ra_transition_factor[1] = 7.1;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x44);               // 浣胯兘闄€铻轰华鏁板瓧浣庨€氭护娉㈠櫒
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL4_C, 0x02);               // 浣胯兘鏁板瓧浣庨€氭护娉㈠櫒
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL5_C, 0x00);               // 鍔犻€熷害璁′笌闄€铻轰华鍥涜垗浜斿叆
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00);               // 寮€鍚姞閫熷害璁￠珮鎬ц兘妯″紡 闄€铻轰华浣庨€氭护娉?133hz
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL7_G, 0x00);               // 寮€鍚檧铻轰华楂樻€ц兘妯″紡 鍏抽棴楂橀€氭护娉?
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL9_XL, 0x01);              // 鍏抽棴I3C鎺ュ彛
        
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);       // 寮€鍚疕UB瀵勫瓨鍣ㄨ闂?鐢ㄤ簬閰嶇疆鍦扮璁?
        imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x80);         // 澶嶄綅I2C涓绘満
        system_delay_ms(2);
        imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x00);         // 娓呴櫎澶嶄綅鏍囧織
        system_delay_ms(2);
        
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x80);// 澶嶄綅杩炴帴鐨勫璁?
        system_delay_ms(2);
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x00);
        system_delay_ms(2);
        
        if(imu963ra_mag_self_check())
        {
            // printf( "IMU963RA mag self check error.\r\n");
            return_state = 1;
            break;
        }
        
        // IMU963RA_MAG_ADDR 瀵勫瓨鍣?
        // 璁剧疆涓?0x09 纾佸姏璁￠噺绋嬩负 2G   鑾峰彇鍒扮殑纾佸姏璁℃暟鎹櫎浠?12000   鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 G(楂樻柉)
        // 璁剧疆涓?0x19 纾佸姏璁￠噺绋嬩负 8G   鑾峰彇鍒扮殑纾佸姏璁℃暟鎹櫎浠?3000    鍙互杞寲涓哄甫鐗╃悊鍗曚綅鐨勬暟鎹?鍗曚綅 G(楂樻柉)
        switch(IMU963RA_MAG_SAMPLE_DEFAULT)
        {
            default:
            {
                // printf( "IMU963RA_MAG_SAMPLE_DEFAULT set error.\r\n");
                return_state = 1;
            }
            break;
            
            case IMU963RA_MAG_SAMPLE_2G:
            {
                imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL1, 0x09);
                imu963ra_transition_factor[2] = 12000;
            }
            break;
            
            case IMU963RA_MAG_SAMPLE_8G:
            {
                imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL1, 0x19);
                imu963ra_transition_factor[2] = 3000;
            }
            break;
        }
        
        if(1 == return_state)
        {
            break;
        }
        
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_FBR, 0x01);
        imu963ra_connect_mag(IMU963RA_MAG_ADDR, IMU963RA_MAG_OUTX_L);
        
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // 鍏抽棴HUB瀵勫瓨鍣ㄨ闂?
        
        system_delay_ms(20);                                                    // 绛夊緟纾佸姏璁¤幏鍙栨暟鎹?
    }
    while(0);
    
    return return_state;
}
