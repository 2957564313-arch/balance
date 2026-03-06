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
*                   SCL                 鏌ョ湅 zf_device_dl1a.h 涓?DL1A_SCL_PIN  瀹忓畾涔?
*                   SDA                 鏌ョ湅 zf_device_dl1a.h 涓?DL1A_SDA_PIN  瀹忓畾涔?
*                   XS                  鏌ョ湅 zf_device_dl1a.h 涓?DL1A_XS_PIN   瀹忓畾涔?
*                   VCC                 5V 鐢垫簮
*                   GND                 鐢垫簮鍦?
*                   ------------------------------------
********************************************************************************************************************/

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"
#include "zf_device_dl1a.h"
#include "zf_device_type.h"

#pragma warning disable = 183

static  uint8   dl1a_init_flag = 0;
uint8 dl1a_finsh_flag = 0;
uint16 dl1a_distance_mm = 8192;

#if (DL1A_USE_INTERFACE==SOFT_IIC)
	static soft_iic_info_struct dl1a_iic_struct;

	#define dl1a_write_array(dat, len)          (soft_iic_write_8bit_array(&dl1a_iic_struct, (dat), (len)))
	#define dl1a_write_register(reg, dat)       (soft_iic_write_8bit_register(&dl1a_iic_struct, (reg), (dat)))
	#define dl1a_read_register(reg)              (soft_iic_read_8bit_register(&dl1a_iic_struct, (reg)))
	#define dl1a_read_registers(reg, dat, len)  (soft_iic_read_8bit_registers(&dl1a_iic_struct, (reg), (dat), (len)))
#elif (DL1A_USE_INTERFACE==HARDWARE_IIC)
	#define dl1a_write_array(dat, len)          (iic_write_8bit_array(DL1A_IIC, DL1A_DEV_ADDR, (dat), (len)))
	#define dl1a_write_register(reg, dat)       (iic_write_8bit_register(DL1A_IIC, DL1A_DEV_ADDR, (reg), (dat)))
	#define dl1a_read_register(reg)              (iic_read_8bit_register(DL1A_IIC, DL1A_DEV_ADDR, (reg)))
	#define dl1a_read_registers(reg, dat, len)  (iic_read_8bit_registers(DL1A_IIC, DL1A_DEV_ADDR, (reg), (dat), (len)))
#endif

// 杩欎釜閫熺巼琛ㄧず浠庣洰鏍囧弽灏勫苟琚澶囨娴嬪埌鐨勪俊鍙风殑鎸箙
// 璁剧疆姝ら檺鍒跺彲浠ョ‘瀹氫紶鎰熷櫒鎶ュ憡鏈夋晥璇绘暟鎵€闇€鐨勬渶灏忔祴閲忓€?
// 璁剧疆涓€涓緝浣庣殑闄愬埗鍙互澧炲姞浼犳劅鍣ㄧ殑娴嬮噺鑼冨洿
// 浣嗕技涔庝篃澧炲姞浜?<鐢变簬鏉ヨ嚜鐩爣浠ュ鐨勭墿浣撶殑涓嶉渶瑕佺殑鍙嶅皠瀵艰嚧> 寰楀埌涓嶅噯纭鏁扮殑鍙兘鎬?
// 榛樿涓?0.25 MCPS 鍙璁捐寖鍥翠负 0 - 511.99
#define DL1A_DEFAULT_RATE_LIMIT  (0.25)

// 浠庡瘎瀛樺櫒鏁版嵁瑙ｇ爜 PCLKs 涓?VCSEL (vertical cavity surface emitting laser) 鐨勮剦瀹藉懆鏈?
#define decode_vcsel_period(reg_val)            (((reg_val) + 1) << 1)

// 浠?PCLK 涓殑 VCSEL 鍛ㄦ湡璁＄畻瀹忓懆鏈?(浠?*绾崇涓哄崟浣?
// PLL_period_ps = 1655
// macro_period_vclks = 2304
#define calc_macro_period(vcsel_period_pclks)   ((((uint32)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇璁惧 SPAD 淇℃伅
// 鍙傛暟璇存槑     index           绱㈠紩
// 鍙傛暟璇存槑     type            绫诲瀷鍊?
// 杩斿洖鍙傛暟     uint8           鏄惁鎴愬姛 0-鎴愬姛 1-澶辫触
// 浣跨敤绀轰緥     dl1a_get_spad_info(index, type_is_aperture);
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_get_spad_info (uint8 *index, uint8 *type_is_aperture)
{
    uint8 tmp = 0;
    uint8 return_state = 0;
    volatile uint16 loop_count = 0;
    
    do
    {
        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);
        
        dl1a_write_register(0xFF, 0x06);
        dl1a_read_registers(0x83, &tmp, 1);
        dl1a_write_register(0x83, tmp | 0x04);
        dl1a_write_register(0xFF, 0x07);
        dl1a_write_register(0x81, 0x01);
        
        dl1a_write_register(0x80, 0x01);
        
        dl1a_write_register(0x94, 0x6b);
        dl1a_write_register(0x83, 0x00);
        
        tmp = 0x00;
        
        while(0x00 == tmp || 0xFF == tmp)
        {
            system_delay_ms(1);
            dl1a_read_registers(0x83, &tmp, 1);
            
            if(DL1A_TIMEOUT_COUNT < loop_count ++)
            {
                return_state = 1;
                break;
            }
            
        }
        
        if(return_state)
        {
            break;
        }
        
        dl1a_write_register(0x83, 0x01);
        dl1a_read_registers(0x92, &tmp, 1);
        
        *index = tmp & 0x7f;
        *type_is_aperture = (tmp >> 7) & 0x01;
        
        dl1a_write_register(0x81, 0x00);
        dl1a_write_register(0xFF, 0x06);
        dl1a_read_registers(0x83, &tmp, 1);
        dl1a_write_register(0x83, tmp);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x01);
        
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);
    }
    while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    灏嗚秴鏃舵暟鍊间粠 MCLKs 杞崲鍒板搴旂殑 ms
// 鍙傛暟璇存槑     timeout_period_mclks    瓒呮椂鍛ㄦ湡 MCLKs
// 鍙傛暟璇存槑     vcsel_period_pclks      PCLK 鍊?
// 杩斿洖鍙傛暟     uint32                  杩斿洖瓒呮椂鏁板€?
// 浣跨敤绀轰緥     dl1a_timeout_mclks_to_microseconds(timeout_period_mclks, vcsel_period_pclks);
// 澶囨敞淇℃伅     灏嗗簭鍒楁楠よ秴鏃朵粠鍏锋湁缁欏畾 VCSEL 鍛ㄦ湡鐨?MCLK (浠?PCLK 涓哄崟浣?杞崲涓哄井绉?
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_timeout_mclks_to_microseconds (uint16 timeout_period_mclks, uint8 vcsel_period_pclks)
{
    uint32 macro_period_ns = calc_macro_period(vcsel_period_pclks);
    
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    灏嗚秴鏃舵暟鍊间粠 ms 杞崲鍒板搴旂殑 MCLKs
// 鍙傛暟璇存槑     timeout_period_us   瓒呮椂鍛ㄦ湡 寰鍗曚綅
// 鍙傛暟璇存槑     vcsel_period_pclks  PCLK 鍊?
// 杩斿洖鍙傛暟     uint32              杩斿洖瓒呮椂鏁板€?
// 浣跨敤绀轰緥     dl1a_timeout_microseconds_to_mclks(timeout_period_us, vcsel_period_pclks);
// 澶囨敞淇℃伅     灏嗗簭鍒楁楠よ秴鏃朵粠寰杞崲涓哄叿鏈夌粰瀹?VCSEL 鍛ㄦ湡鐨?MCLK (浠?PCLK 涓哄崟浣?
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_timeout_microseconds_to_mclks (uint32 timeout_period_us, uint8 vcsel_period_pclks)
{
    uint32 macro_period_ns = calc_macro_period(vcsel_period_pclks);
    
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    瀵硅秴鏃舵暟鍊艰繘琛岃В鐮?
// 鍙傛暟璇存槑     reg_val         瓒呮椂鏃堕暱 瀵勫瓨鍣ㄥ€?
// 杩斿洖鍙傛暟     uint16          杩斿洖瓒呮椂鏁板€?
// 浣跨敤绀轰緥     dl1a_decode_timeout(reg_val);
// 澶囨敞淇℃伅     浠庡瘎瀛樺櫒鍊艰В鐮?MCLK 涓殑搴忓垪姝ラ瓒呮椂
//-------------------------------------------------------------------------------------------------------------------
static uint16 dl1a_decode_timeout (uint16 reg_val)
{
    // 鏍煎紡: (LSByte * 2 ^ MSByte) + 1
    return  (uint16)((reg_val & 0x00FF) <<
                     (uint16)((reg_val & 0xFF00) >> 8)) + 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    瀵硅秴鏃舵暟鍊艰繘琛岀紪鐮?
// 鍙傛暟璇存槑     timeout_mclks   瓒呮椂鏃堕暱 -MCLKs 鍊?
// 杩斿洖鍙傛暟     uint16          杩斿洖缂栫爜鍊?
// 浣跨敤绀轰緥     dl1a_encode_timeout(timeout_mclks);
// 澶囨敞淇℃伅     鍦?MCLK 涓瓒呮椂鐨勫簭鍒楁楠よ秴鏃跺瘎瀛樺櫒鍊艰繘琛岀紪鐮?
//-------------------------------------------------------------------------------------------------------------------
static uint16 dl1a_encode_timeout (uint16 timeout_mclks)
{
    uint32 edata ls_byte = 0;
    uint16 edata ms_byte = 0;
    uint16 edata return_data = 0;
    
    if (timeout_mclks > 0)
    {
        // 鏍煎紡: (LSByte * 2 ^ MSByte) + 1
        ls_byte = timeout_mclks - 1;
        
        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        
        return_data = (ms_byte << 8) | (ls_byte & 0xFF);
    }
    
    return return_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇搴忓垪姝ラ浣胯兘璁剧疆
// 鍙傛暟璇存槑     enables         搴忓垪浣胯兘姝ラ缁撴瀯浣?
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     dl1a_get_sequence_step_enables(enables);
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_get_sequence_step_enables(dl1a_sequence_enables_step_struct *enables)
{
    uint8 sequence_config = 0;
    dl1a_read_registers(DL1A_SYSTEM_SEQUENCE_CONFIG, &sequence_config, 1);
    
    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇鑴夊啿鍛ㄦ湡
// 鍙傛暟璇存槑     type            棰勯噺绋嬬被鍨?
// 杩斿洖鍙傛暟     uint8           杩斿洖鐨勫懆鏈熷€?
// 浣跨敤绀轰緥     dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);
// 澶囨敞淇℃伅     鍦?PCLKs 涓幏鍙栫粰瀹氬懆鏈熺被鍨嬬殑 VCSEL 鑴夊啿鍛ㄦ湡
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_get_vcsel_pulse_period (dl1a_vcsel_period_type_enum type)
{
    uint8 data_buffer = 0;
    
    if(DL1A_VCSEL_PERIOD_PER_RANGE == type)
    {
        dl1a_read_registers(DL1A_PRE_RANGE_CONFIG_VCSEL_PERIOD, &data_buffer, 1);
        data_buffer = decode_vcsel_period(data_buffer);
    }
    else if(DL1A_VCSEL_PERIOD_FINAL_RANGE == type)
    {
        dl1a_read_registers(DL1A_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &data_buffer, 1);
        data_buffer = decode_vcsel_period(data_buffer);
    }
    else
    {
        data_buffer = 255;
    }
    
    return data_buffer;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇搴忓垪姝ラ瓒呮椂璁剧疆
// 鍙傛暟璇存槑     enables         搴忓垪浣胯兘姝ラ缁撴瀯浣?
// 鍙傛暟璇存槑     timeouts        搴忓垪瓒呮椂姝ラ缁撴瀯浣?
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     dl1a_get_sequence_step_timeouts(enables, timeouts);
// 澶囨敞淇℃伅     鑾峰彇鎵€鏈夎秴鏃惰€屼笉浠呬粎鏄姹傜殑瓒呮椂 骞朵笖杩樺瓨鍌ㄤ腑闂村€?
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_get_sequence_step_timeouts (dl1a_sequence_enables_step_struct const *enables, dl1a_sequence_timeout_step_struct *timeouts)
{
    uint8 reg_buffer[2];
    uint16 reg16_buffer = 0;
    
    timeouts->pre_range_vcsel_period_pclks = dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);
    
    dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, reg_buffer, 1);
    timeouts->msrc_dss_tcc_mclks = reg_buffer[0] + 1;
    timeouts->msrc_dss_tcc_us = dl1a_timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks, (uint8)timeouts->pre_range_vcsel_period_pclks);
    
    dl1a_read_registers(DL1A_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, reg_buffer, 2);
    reg16_buffer = ((uint16) reg_buffer[0] << 8) | reg_buffer[1];
    timeouts->pre_range_mclks = dl1a_decode_timeout(reg16_buffer);
    timeouts->pre_range_us = dl1a_timeout_mclks_to_microseconds(timeouts->pre_range_mclks, (uint8)timeouts->pre_range_vcsel_period_pclks);
    
    timeouts->final_range_vcsel_period_pclks = dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_FINAL_RANGE);
    
    dl1a_read_registers(DL1A_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, reg_buffer, 2);
    reg16_buffer = ((uint16) reg_buffer[0] << 8) | reg_buffer[1];
    timeouts->final_range_mclks = dl1a_decode_timeout(reg16_buffer);
    
    if(enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }
    
    timeouts->final_range_us = dl1a_timeout_mclks_to_microseconds(timeouts->final_range_mclks, (uint8)timeouts->final_range_vcsel_period_pclks);
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鎵ц鍗曟鍙傝€冩牎鍑?
// 鍙傛暟璇存槑     vhv_init_byte   棰勮鏍″噯鍊?
// 杩斿洖鍙傛暟     uint8           鎿嶄綔鏄惁鎴愬姛 0-鎴愬姛 1-澶辫触
// 浣跨敤绀轰緥     dl1a_get_vcsel_pulse_period(DL1A_VCSEL_PERIOD_PER_RANGE);
// 澶囨敞淇℃伅     鍦?PCLKs 涓幏鍙栫粰瀹氬懆鏈熺被鍨嬬殑 VCSEL 鑴夊啿鍛ㄦ湡
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_perform_single_ref_calibration (uint8 vhv_init_byte)
{
    uint8 return_state = 0;
    uint8 data_buffer = 0;
    volatile uint16 loop_count = 0;
    
    do
    {
        dl1a_write_register(DL1A_SYSRANGE_START, 0x01 | vhv_init_byte);
        dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, &data_buffer, 1);
        
        while(0 == (data_buffer & 0x07))
        {
            system_delay_ms(1);
            dl1a_read_registers(DL1A_MSRC_CONFIG_TIMEOUT_MACROP, &data_buffer, 1);
            
            if(DL1A_TIMEOUT_COUNT < loop_count ++)
            {
                return_state = 1;
                break;
            }
        }
        
        if(return_state)
        {
            break;
        }
        
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
        dl1a_write_register(DL1A_SYSRANGE_START, 0x00);
    }
    while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    璁剧疆娴嬮噺瀹氭椂棰勭畻 (浠ュ井绉掍负鍗曚綅)
// 鍙傛暟璇存槑     budget_us       璁惧畾鐨勬祴閲忓厑璁哥殑鏃堕棿
// 杩斿洖鍙傛暟     uint8           鎿嶄綔缁撴灉 0-鎴愬姛 1-澶辫触
// 浣跨敤绀轰緥     dl1a_set_measurement_timing_budget(measurement_timing_budget_us);
// 澶囨敞淇℃伅     杩欐槸涓€娆℃祴閲忓厑璁哥殑鏃堕棿
//              鍗冲湪娴嬭窛搴忓垪鐨勫瓙姝ラ涔嬮棿鍒嗛厤鏃堕棿棰勭畻
//              鏇撮暱鐨勬椂闂撮绠楀厑璁告洿绮剧‘鐨勬祴閲?
//              澧炲姞涓€涓狽鍊嶇殑棰勭畻鍙互鍑忓皯涓€涓猻qrt(N)鍊嶇殑鑼冨洿娴嬮噺鏍囧噯鍋忓樊
//              榛樿涓?3姣 鏈€灏忓€间负20 ms
//-------------------------------------------------------------------------------------------------------------------
static uint8 dl1a_set_measurement_timing_budget (uint32 budget_us)
{
    uint8 return_state = 0;
    uint8 data_buffer[3];
    uint16 dat = 0;
    
    uint32 final_range_timeout_us = 0;
    uint16 final_range_timeout_mclks = 0;
    
    dl1a_sequence_enables_step_struct enables;
    dl1a_sequence_timeout_step_struct timeouts;
    
    do
    {
        uint32 used_budget_us = DL1A_SET_START_OVERHEAD + DL1A_END_OVERHEAD;
        
        if(DL1A_MIN_TIMING_BUDGET > budget_us)
        {
            return_state = 1;
            break;
        }
        
        dl1a_get_sequence_step_enables(&enables);
        dl1a_get_sequence_step_timeouts(&enables, &timeouts);
        
        if(enables.tcc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + DL1A_TCC_OVERHEAD);
        }
        
        if(enables.dss)
        {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DL1A_DSS_OVERHEAD);
        }
        else if(enables.msrc)
        {
            used_budget_us += (timeouts.msrc_dss_tcc_us + DL1A_MSRC_OVERHEAD);
        }
        
        if(enables.pre_range)
        {
            used_budget_us += (timeouts.pre_range_us + DL1A_PRERANGE_OVERHEAD);
        }
        
        if(enables.final_range)
        {
            // 璇锋敞鎰?鏈€缁堣寖鍥磋秴鏃剁敱璁℃椂棰勭畻鍜屽簭鍒椾腑鎵€鏈夊叾浠栬秴鏃剁殑鎬诲拰鍐冲畾
            // 濡傛灉娌℃湁绌洪棿鐢ㄤ簬鏈€缁堣寖鍥磋秴鏃?鍒欏皢璁剧疆閿欒
            // 鍚﹀垯 鍓╀綑鏃堕棿灏嗗簲鐢ㄤ簬鏈€缁堣寖鍥?
            used_budget_us += DL1A_FINALlRANGE_OVERHEAD;
            
            if(used_budget_us > budget_us)
            {
                // 璇锋眰鐨勮秴鏃跺お澶?
                return_state = 1;
                break;
            }
            
            // 瀵逛簬鏈€缁堣秴鏃惰寖鍥?蹇呴』娣诲姞棰勯噺绋嬭寖鍥磋秴鏃?
            // 涓烘 鏈€缁堣秴鏃跺拰棰勯噺绋嬭秴鏃跺繀椤讳互瀹忓懆鏈?MClks 琛ㄧず
            // 鍥犱负瀹冧滑鍏锋湁涓嶅悓鐨?VCSEL 鍛ㄦ湡
            final_range_timeout_us = budget_us - used_budget_us;
            final_range_timeout_mclks =
                (uint16)dl1a_timeout_microseconds_to_mclks(final_range_timeout_us,
                    (uint8)timeouts.final_range_vcsel_period_pclks);
                    
            if(enables.pre_range)
            {
                final_range_timeout_mclks += timeouts.pre_range_mclks;
            }
            
            dat = dl1a_encode_timeout(final_range_timeout_mclks);
            data_buffer[0] = DL1A_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI;
            data_buffer[1] = ((dat >> 8) & 0xFF);
            data_buffer[2] = (dat & 0xFF);
            dl1a_write_array(data_buffer, 3);
        }
    }
    while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    鑾峰彇娴嬮噺瀹氭椂棰勭畻 (浠ュ井绉掍负鍗曚綅)
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint32          宸茶瀹氱殑娴嬮噺鍏佽鐨勬椂闂?
// 浣跨敤绀轰緥     dl1a_get_measurement_timing_budget();
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
static uint32 dl1a_get_measurement_timing_budget (void)
{
    dl1a_sequence_enables_step_struct enables;
    dl1a_sequence_timeout_step_struct timeouts;
    
    // 寮€濮嬪拰缁撴潫寮€閿€鏃堕棿濮嬬粓瀛樺湪
    uint32 budget_us = DL1A_GET_START_OVERHEAD + DL1A_END_OVERHEAD;
    
    dl1a_get_sequence_step_enables(&enables);
    dl1a_get_sequence_step_timeouts(&enables, &timeouts);
    
    if(enables.tcc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + DL1A_TCC_OVERHEAD);
    }
    
    if(enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DL1A_DSS_OVERHEAD);
    }
    else if(enables.msrc)
    {
        budget_us += (timeouts.msrc_dss_tcc_us + DL1A_MSRC_OVERHEAD);
    }
    
    if(enables.pre_range)
    {
        budget_us += (timeouts.pre_range_us + DL1A_PRERANGE_OVERHEAD);
    }
    
    if(enables.final_range)
    {
        budget_us += (timeouts.final_range_us + DL1A_FINALlRANGE_OVERHEAD);
    }
    
    return budget_us;
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    璁剧疆杩斿洖淇″彿閫熺巼闄愬埗 璇ュ€煎崟浣嶄负 MCPS (鐧句竾娆℃瘡绉?
// 鍙傛暟璇存槑     limit_mcps      璁剧疆鐨勬渶灏忛€熺巼
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     dl1a_set_signal_rate_limit(0.25);
// 澶囨敞淇℃伅     杩欎釜閫熺巼琛ㄧず浠庣洰鏍囧弽灏勫苟琚澶囨娴嬪埌鐨勪俊鍙风殑鎸箙
//              璁剧疆姝ら檺鍒跺彲浠ョ‘瀹氫紶鎰熷櫒鎶ュ憡鏈夋晥璇绘暟鎵€闇€鐨勬渶灏忔祴閲忓€?
//              璁剧疆涓€涓緝浣庣殑闄愬埗鍙互澧炲姞浼犳劅鍣ㄧ殑娴嬮噺鑼冨洿
//              浣嗕技涔庝篃澧炲姞浜?<鐢变簬鏉ヨ嚜鐩爣浠ュ鐨勭墿浣撶殑涓嶉渶瑕佺殑鍙嶅皠瀵艰嚧> 寰楀埌涓嶅噯纭鏁扮殑鍙兘鎬?
//              榛樿涓?0.25 MCPS 鍙璁捐寖鍥翠负 0 - 511.99
//-------------------------------------------------------------------------------------------------------------------
static void dl1a_set_signal_rate_limit (float limit_mcps)
{
    uint8 data_buffer[3];
    uint16 limit_mcps_16bit = (uint16)(limit_mcps * (1 << 7));
    
    zf_assert(0 <= limit_mcps || 511.99 >= limit_mcps);
    
    data_buffer[0] = DL1A_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT;
    data_buffer[1] = ((limit_mcps_16bit >> 8) & 0xFF);
    data_buffer[2] = (limit_mcps_16bit & 0xFF);
    
    dl1a_write_array(data_buffer, 3);
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    杩斿洖浠ユ绫充负鍗曚綅鐨勮寖鍥磋鏁?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     dl1a_get_distance();
// 澶囨敞淇℃伅     鍦ㄥ紑濮嬪崟娆″皠绋嬫祴閲忓悗涔熻皟鐢ㄦ鍑芥暟
//-------------------------------------------------------------------------------------------------------------------
void dl1a_get_distance (void)
{
    if(dl1a_init_flag)
    {
        uint8 reg_databuffer[3];
        
        dl1a_read_registers(DL1A_RESULT_INTERRUPT_STATUS, reg_databuffer, 1);
        
        if(0 != (reg_databuffer[0] & 0x07))
        {
            // 鍋囪绾挎€у害鏍℃澧炵泭涓洪粯璁ゅ€?1000 涓旀湭鍚敤鍒嗘暟鑼冨洿
            dl1a_read_registers(DL1A_RESULT_RANGE_STATUS + 10, reg_databuffer, 2);
            dl1a_distance_mm = ((uint16)reg_databuffer[0] << 8);
            dl1a_distance_mm |= reg_databuffer[1];
            
            dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
            dl1a_finsh_flag = 1;
        }
        
        if(reg_databuffer[0] & 0x10)
        {
            dl1a_read_registers(DL1A_RESULT_RANGE_STATUS + 10, reg_databuffer, 2);
            dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    DL1A INT 涓柇鍝嶅簲澶勭悊鍑芥暟
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     dl1a_int_handler();
// 澶囨敞淇℃伅     鏈嚱鏁伴渶瑕佸湪 DL1A_INT_PIN 瀵瑰簲鐨勫閮ㄤ腑鏂鐞嗗嚱鏁颁腑璋冪敤
//-------------------------------------------------------------------------------------------------------------------
void dl1a_int_handler (void)
{
#if DL1A_INT_ENABLE
    dl1a_get_distance();
#endif
}
// 鍑芥暟绠€浠?    鍒濆鍖?DL1A
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     uint8           1-鍒濆鍖栧け璐?0-鍒濆鍖栨垚鍔?
// 浣跨敤绀轰緥     dl1a_init();
// 澶囨敞淇℃伅
//-------------------------------------------------------------------------------------------------------------------
uint8 dl1a_init (void)
{
    uint32 measurement_timing_budget_us;
    uint8 stop_variable = 0;
    uint8 return_state = 0;
    uint8 reg_data_buffer = 0;
    uint8 ref_spad_map[6];
    uint8 data_buffer[7];
    uint16 i = 0;
    
    uint8 temp_buff1[][2] =  	// BUFF1
    {
        {0x88, 0x00},
        {0x80, 0x01},
        {0xFF, 0x01},
        {0x00, 0x00},
    };
    
    uint8 temp_buff2[][2] =  	// BUFF2
    {
        {0x00, 0x01},
        {0xFF, 0x00},
        {0x80, 0x00},
    };
    
    uint8 temp_buff3[][2] =  	// BUFF3
    {
        {0xFF, 0x01},
        {DL1A_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00},
        {DL1A_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C	},
        {0xFF, 0x00},
        {DL1A_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4},
    };
    
    uint8 temp_buff4[][2] =  	// BUFF4
    {
        {0xFF, 0x01},
        {0x00, 0x00},
        {0xFF, 0x00},
        {0x09, 0x00},
        {0x10, 0x00},
        {0x11, 0x00},
        {0x24, 0x01},
        {0x25, 0xFF},
        {0x75, 0x00},
        {0xFF, 0x01},
        {0x4E, 0x2C},
        {0x48, 0x00},
        {0x30, 0x20},
        {0xFF, 0x00},
        {0x30, 0x09},
        {0x54, 0x00},
        {0x31, 0x04},
        {0x32, 0x03},
        {0x40, 0x83},
        {0x46, 0x25},
        {0x60, 0x00},
        {0x27, 0x00},
        {0x50, 0x06},
        {0x51, 0x00},
        {0x52, 0x96},
        {0x56, 0x08},
        {0x57, 0x30},
        {0x61, 0x00},
        {0x62, 0x00},
        {0x64, 0x00},
        {0x65, 0x00},
        {0x66, 0xA0},
        {0xFF, 0x01},
        {0x22, 0x32},
        {0x47, 0x14},
        {0x49, 0xFF},
        {0x4A, 0x00},
        {0xFF, 0x00},
        {0x7A, 0x0A},
        {0x7B, 0x00},
        {0x78, 0x21},
        {0xFF, 0x01},
        {0x23, 0x34},
        {0x42, 0x00},
        {0x44, 0xFF},
        {0x45, 0x26},
        {0x46, 0x05},
        {0x40, 0x40},
        {0x0E, 0x06},
        {0x20, 0x1A},
        {0x43, 0x40},
        {0xFF, 0x00},
        {0x34, 0x03},
        {0x35, 0x44},
        {0xFF, 0x01},
        {0x31, 0x04},
        {0x4B, 0x09},
        {0x4C, 0x05},
        {0x4D, 0x04},
        {0xFF, 0x00},
        {0x44, 0x00},
        {0x45, 0x20},
        {0x47, 0x08},
        {0x48, 0x28},
        {0x67, 0x00},
        {0x70, 0x04},
        {0x71, 0x01},
        {0x72, 0xFE},
        {0x76, 0x00},
        {0x77, 0x00},
        {0xFF, 0x01},
        {0x0D, 0x01},
        {0xFF, 0x00},
        {0x80, 0x01},
        {0x01, 0xF8},
        {0xFF, 0x01},
        {0x8E, 0x01},
        {0x00, 0x01},
        {0xFF, 0x00},
        {0x80, 0x00}
    };
    
    memset(ref_spad_map, 0, 6);
    memset(data_buffer, 0, 7);
    
#if (DL1A_USE_INTERFACE==SOFT_IIC)
    soft_iic_init(&dl1a_iic_struct, DL1A_DEV_ADDR, DL1A_SOFT_IIC_DELAY, DL1A_SCL_PIN, DL1A_SDA_PIN);
#elif (DL1A_USE_INTERFACE==HARDWARE_IIC)
    iic_init(DL1A_IIC, DL1A_DEV_ADDR, DL1A_IIC_SPEED, DL1A_SCL_PIN, DL1A_SDA_PIN);
#endif
    
#if DL1A_XS_ENABLE
    gpio_init(DL1A_XS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
#endif
    
    do
    {
    
#if DL1A_XS_ENABLE
        system_delay_ms(100);
        gpio_low(DL1A_XS_PIN);
        system_delay_ms(50);
        gpio_high(DL1A_XS_PIN);
        system_delay_ms(100);
#endif
        
        // -------------------------------- DL1A 鍚姩鍒濆鍖?--------------------------------
        reg_data_buffer = dl1a_read_register(DL1A_IO_VOLTAGE_CONFIG);         // 浼犳劅鍣ㄩ粯璁?IO 涓?1.8V 妯″紡
        dl1a_write_register(DL1A_IO_VOLTAGE_CONFIG, reg_data_buffer | 0x01);  // 閰嶇疆 IO 涓?2.8V 妯″紡
        
        for(i = 0; i < (sizeof(temp_buff1) / 2); i++)
        {
            dl1a_write_register(temp_buff1[i][0], temp_buff1[i][1]);
        }
        
        //        dl1a_write_register(0x88, 0x00);                                         // 璁剧疆涓烘爣鍑?IIC 妯″紡
        //        dl1a_write_register(0x80, 0x01);
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x00, 0x00);
        
        dl1a_read_registers(0x91, &stop_variable, 1);
        
        for(i = 0; i < (sizeof(temp_buff2) / 2); i++)
        {
            dl1a_write_register(temp_buff2[i][0], temp_buff2[i][1]);
        }
        
        //        dl1a_write_register(0x00, 0x01);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x80, 0x00);
        
        // 绂佺敤 SIGNAL_RATE_MSRC(bit1) 鍜?SIGNAL_RATE_PRE_RANGE(bit4) 闄愬埗妫€鏌?
        reg_data_buffer = dl1a_read_register(DL1A_MSRC_CONFIG);
        dl1a_write_register(DL1A_MSRC_CONFIG, reg_data_buffer | 0x12);
        
        dl1a_set_signal_rate_limit(DL1A_DEFAULT_RATE_LIMIT);                  // 璁剧疆淇″彿閫熺巼闄愬埗
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xFF);
        // -------------------------------- DL1A 鍚姩鍒濆鍖?--------------------------------
        
        // -------------------------------- DL1A 閰嶇疆鍒濆鍖?--------------------------------
        if(dl1a_get_spad_info(&data_buffer[0], &data_buffer[1]))
        {
            return_state = 1;
            // printf( "DL1A self check error.\r\n");
            break;
        }
        
        // 浠?GLOBAL_CONFIG_SPAD_ENABLES_REF_[0-6] 鑾峰彇 SPAD map (RefGoodSpadMap) 鏁版嵁
        dl1a_read_registers(DL1A_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
        
        for(i = 0; i < (sizeof(temp_buff3) / 2); i++)
        {
            dl1a_write_register(temp_buff3[i][0], temp_buff3[i][1]);
        }
        
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(DL1A_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        //        dl1a_write_register(DL1A_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(DL1A_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
        
        data_buffer[2] = data_buffer[1] ? 12 : 0; // 12 is the first aperture spad
        
        for(i = 0; 48 > i; i ++)
        {
            if(i < data_buffer[2] || data_buffer[3] == data_buffer[0])
            {
                // 姝や綅浣庝簬搴斿惎鐢ㄧ殑绗竴涓綅
                // 鎴栬€?(eference_spad_count) 浣嶅凡鍚敤
                // 鍥犳姝や綅涓洪浂
                ref_spad_map[i / 8] &= ~(1 << (i % 8));
            }
            else if((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            {
                data_buffer[3] ++;
            }
        }
        
        data_buffer[0] = DL1A_GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
        
        for(i = 1; 7 > i; i ++)
        {
            data_buffer[1] = ref_spad_map[i - 1];
        }
        
        dl1a_write_array(data_buffer, 7);
        
        for(i = 0; i < (sizeof(temp_buff4) / 2); i++)
        {
            dl1a_write_register(temp_buff4[i][0], temp_buff4[i][1]);
        }
        
        //        // 榛樿杞崲璁剧疆 version 02/11/2015_v36
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x00, 0x00);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x09, 0x00);
        //        dl1a_write_register(0x10, 0x00);
        //        dl1a_write_register(0x11, 0x00);
        //        dl1a_write_register(0x24, 0x01);
        //        dl1a_write_register(0x25, 0xFF);
        //        dl1a_write_register(0x75, 0x00);
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x4E, 0x2C);
        //        dl1a_write_register(0x48, 0x00);
        //        dl1a_write_register(0x30, 0x20);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x30, 0x09);
        //        dl1a_write_register(0x54, 0x00);
        //        dl1a_write_register(0x31, 0x04);
        //        dl1a_write_register(0x32, 0x03);
        //        dl1a_write_register(0x40, 0x83);
        //        dl1a_write_register(0x46, 0x25);
        //        dl1a_write_register(0x60, 0x00);
        //        dl1a_write_register(0x27, 0x00);
        //        dl1a_write_register(0x50, 0x06);
        //        dl1a_write_register(0x51, 0x00);
        //        dl1a_write_register(0x52, 0x96);
        //        dl1a_write_register(0x56, 0x08);
        //        dl1a_write_register(0x57, 0x30);
        //        dl1a_write_register(0x61, 0x00);
        //        dl1a_write_register(0x62, 0x00);
        //        dl1a_write_register(0x64, 0x00);
        //        dl1a_write_register(0x65, 0x00);
        //        dl1a_write_register(0x66, 0xA0);
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x22, 0x32);
        //        dl1a_write_register(0x47, 0x14);
        //        dl1a_write_register(0x49, 0xFF);
        //        dl1a_write_register(0x4A, 0x00);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x7A, 0x0A);
        //        dl1a_write_register(0x7B, 0x00);
        //        dl1a_write_register(0x78, 0x21);
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x23, 0x34);
        //        dl1a_write_register(0x42, 0x00);
        //        dl1a_write_register(0x44, 0xFF);
        //        dl1a_write_register(0x45, 0x26);
        //        dl1a_write_register(0x46, 0x05);
        //        dl1a_write_register(0x40, 0x40);
        //        dl1a_write_register(0x0E, 0x06);
        //        dl1a_write_register(0x20, 0x1A);
        //        dl1a_write_register(0x43, 0x40);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x34, 0x03);
        //        dl1a_write_register(0x35, 0x44);
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x31, 0x04);
        //        dl1a_write_register(0x4B, 0x09);
        //        dl1a_write_register(0x4C, 0x05);
        //        dl1a_write_register(0x4D, 0x04);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x44, 0x00);
        //        dl1a_write_register(0x45, 0x20);
        //        dl1a_write_register(0x47, 0x08);
        //        dl1a_write_register(0x48, 0x28);
        //        dl1a_write_register(0x67, 0x00);
        //        dl1a_write_register(0x70, 0x04);
        //        dl1a_write_register(0x71, 0x01);
        //        dl1a_write_register(0x72, 0xFE);
        //        dl1a_write_register(0x76, 0x00);
        //        dl1a_write_register(0x77, 0x00);
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x0D, 0x01);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x80, 0x01);
        //        dl1a_write_register(0x01, 0xF8);
        //        dl1a_write_register(0xFF, 0x01);
        //        dl1a_write_register(0x8E, 0x01);
        //        dl1a_write_register(0x00, 0x01);
        //        dl1a_write_register(0xFF, 0x00);
        //        dl1a_write_register(0x80, 0x00);
        
        // 灏嗕腑鏂厤缃缃负鏂版牱鍝佸氨缁?
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_GPIO_CONFIG, 0x04);
        reg_data_buffer = dl1a_read_register(DL1A_GPIO_HV_MUX_ACTIVE_HIGH);
        dl1a_write_register(DL1A_GPIO_HV_MUX_ACTIVE_HIGH, reg_data_buffer & ~0x10);
        dl1a_write_register(DL1A_SYSTEM_INTERRUPT_CLEAR, 0x01);
        
        measurement_timing_budget_us  = dl1a_get_measurement_timing_budget();
        
        // 榛樿鎯呭喌涓嬬鐢?MSRC 鍜?TCC
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xE8);
        dl1a_set_measurement_timing_budget(measurement_timing_budget_us);    // 閲嶆柊璁＄畻鏃跺簭棰勭畻
        // -------------------------------- DL1A 閰嶇疆鍒濆鍖?--------------------------------
        
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0x01);
        
        if(dl1a_perform_single_ref_calibration(0x40))
        {
            return_state = 1;
            // printf( "DL1A perform single reference calibration error.\r\n");
            break;
        }
        
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0x02);
        
        if(dl1a_perform_single_ref_calibration(0x00))
        {
            return_state = 1;
            // printf( "DL1A perform single reference calibration error.\r\n");
            break;
        }
        
        dl1a_write_register(DL1A_SYSTEM_SEQUENCE_CONFIG, 0xE8);           // 鎭㈠浠ュ墠鐨勫簭鍒楅厤缃?
        
        system_delay_ms(100);
        
        dl1a_write_register(0x80, 0x01);
        dl1a_write_register(0xFF, 0x01);
        dl1a_write_register(0x00, 0x00);
        dl1a_write_register(0x91, stop_variable);
        dl1a_write_register(0x00, 0x01);
        dl1a_write_register(0xFF, 0x00);
        dl1a_write_register(0x80, 0x00);
        
        dl1a_write_register(DL1A_SYSRANGE_START, 0x02);
        dl1a_init_flag = 1;
        
#if DL1A_INT_ENABLE
        exti_init(DL1A_INT_PIN, EXTI_TRIGGER_FALLING);
        dl1a_int_handler();
        dl1a_finsh_flag = 0;
#endif
        set_tof_type(TOF_DL1A, dl1a_int_handler);
    }
    while(0);
    
    return return_state;
}
