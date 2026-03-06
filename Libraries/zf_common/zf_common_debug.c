#include "zf_common_fifo.h"
#include "zf_common_debug.h"
#include "zf_common_clock.h"
#include "zf_common_interrupt.h"
#include "zf_common_typedef.h"

#include "zf_driver_uart.h"
#include "zf_driver_delay.h"

#pragma warning disable = 183
#pragma warning disable = 177

#if DEBUG_UART_USE_INTERRUPT                                                    // 濡傛灉鍚敤 debug uart 鎺ユ敹涓柇
uint8                       debug_uart_buffer[DEBUG_RING_BUFFER_LEN];           // 鏁版嵁瀛樻斁鏁扮粍
#endif

fifo_struct                 debug_uart_fifo;

//static debug_output_struct  debug_output_info;
static volatile uint8       zf_debug_init_flag = 1;
static volatile uint8       zf_debug_assert_enable = 1;

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?     debug 杞欢鏃跺嚱鏁?鍦?120MHz 涓嬫槸涓€绉掑鐨勬椂闂?鍚勫崟鐗囨満闇€瑕佹牴鎹悇鑷椂閽熻瘯楠?
//// 鍙傛暟璇存槑     pass        鍒ゆ柇鏄惁瑙﹀彂鏂█
//// 鍙傛暟璇存槑     *file       鏂囦欢鍚?
//// 鍙傛暟璇存槑     line        鐩爣琛屾暟
//// 杩斿洖鍙傛暟     void
////-------------------------------------------------------------------------------------------------------------------
//static void debug_delay (void)
//{
//    vuint32 loop_1 = 0, loop_2 = 0;
//    for(loop_1 = 0; loop_1 <= 0xFF; loop_1 ++)
//        for(loop_2 = 0; loop_2 <= 0x1FF; loop_2 ++)
//            _nop_();
//}


////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?    debug 淇濇姢澶勭悊 涓昏鏄槻姝㈡柇瑷€鍚庡嚭鐜颁俊鍙风淮鎸佽€屽鑷寸‖浠跺け鎺?
//// 鍙傛暟璇存槑     void
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥     debug_protective_handler();
//// 澶囨敞淇℃伅     鏈嚱鏁板湪鏂囦欢鍐呴儴璋冪敤 鐢ㄦ埛涓嶇敤鍏虫敞 涔熶笉鍙慨鏀?
////-------------------------------------------------------------------------------------------------------------------
//static void debug_protective_handler (void)
//{
//   // 鏆傛湭鏇存柊
//}

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?     debug 涓插彛杈撳嚭鎺ュ彛 姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
//// 鍙傛暟璇存槑     *str        闇€瑕佽緭鍑虹殑瀛楃涓?
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥     debug_uart_str_output("Log message");
//// 澶囨敞淇℃伅     鏈嚱鏁板湪鏂囦欢鍐呴儴璋冪敤 鐢ㄦ埛涓嶇敤鍏虫敞 涔熶笉鍙慨鏀?
////-------------------------------------------------------------------------------------------------------------------
//static void debug_uart_str_output (const char *str)
//{
//    uart_write_string(DEBUG_UART_INDEX, str);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?    debug 杈撳嚭鎺ュ彛
//// 鍙傛暟璇存槑     *type       log 绫诲瀷
//// 鍙傛暟璇存槑     *file       鏂囦欢鍚?
//// 鍙傛暟璇存槑     line        鐩爣琛屾暟
//// 鍙傛暟璇存槑     *str        淇℃伅
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥     debug_output("Log message", file, line, str);
//// 澶囨敞淇℃伅     鏈嚱鏁板湪鏂囦欢鍐呴儴璋冪敤 鐢ㄦ埛涓嶇敤鍏虫敞 涔熶笉鍙慨鏀?
////-------------------------------------------------------------------------------------------------------------------
//static void debug_output (char *type, char *file, int line, char *str)
//{
//    char *file_str;

//    vuint16 i = 0, j = 0;
//    vint16 len_origin = 0;
//    vuint16 show_len = 0;
//    vint16 show_line_index = 0;
//	
//	volatile char output_buffer[256] = {0};
//    volatile char file_path_buffer[64] = {0};

//    len_origin = strlen(file);



//    if(debug_output_info.type_index)
//    {
//        debug_output_info.output_screen_clear();
//    }

//    if(zf_debug_init_flag)
//    {
//        if(debug_output_info.type_index)
//        {
//            // 闇€瑕佸垎琛屽皢鏂囦欢鐨勮矾寰勫拰琛屾暟杈撳嚭
//            // <涓嶈緭鍑哄畬鏁磋矾寰?鍙緭鍑轰竴绾х洰褰?渚嬪 src/main.c>
//            // 杈撳嚭 line : xxxx
//            debug_output_info.output_screen(0, show_line_index ++, type);

//            file_str = file;
//            len_origin = strlen(file);
//            show_len = (debug_output_info.display_x_max / debug_output_info.font_x_size);

//            while(*file_str++ != '\0');

//            // 鍙彇涓€绾х洰褰?濡傛灉鏂囦欢鏀惧湪鐩樼鏍圭洰褰?鎴栬€?MDK 鐨勫伐绋嬫牴鐩綍 灏变細鐩存帴杈撳嚭褰撳墠鐩綍
//            for(j = 0; (j < 2) && (len_origin >= 0); len_origin --)             // 鏌ユ壘涓や釜 '/'
//            {
//                file_str --;
//                if((*file_str == '/') || (*file_str == 0x5C))
//                {
//                    j ++;
//                }
//            }

//            // 鏂囦欢璺緞淇濆瓨鍒版暟缁勪腑
//            if(len_origin >= 0)
//            {
//                file_str ++;
//                sprintf(output_buffer, "file: %s", file_str);
//            }
//            else
//            {
//                if(0 == j)
//                {
//                    sprintf(output_buffer, "file: mdk/%s", file_str);
//                }
//                else
//                {
//                    sprintf(output_buffer, "file: %s", file_str);
//                }
//            }

//            // 灞忓箷鏄剧ず璺緞
//            for(i = 0; i < ((strlen(output_buffer) / show_len) + 1); i ++)
//            {
//                for(j = 0; j < show_len; j ++)
//                {
//                    if(strlen(output_buffer) < (j + i * show_len))
//                    {
//                        break;
//                    }
//                    file_path_buffer[j] = output_buffer[j + i * show_len];
//                }

//                file_path_buffer[j] = '\0';                                     // 鏈熬娣诲姞\0

//                debug_output_info.output_screen(0, debug_output_info.font_y_size * show_line_index ++, file_path_buffer);
//            }

//            // 灞忓箷鏄剧ず琛屽彿
//            sprintf(output_buffer, "line: %d", line);
//            debug_output_info.output_screen(0, debug_output_info.font_y_size * show_line_index ++, output_buffer);

//            // 灞忓箷鏄剧ず Log 濡傛灉鏈夌殑璇?
//            if(NULL != str)
//            {
//                for(i = 0; i < ((strlen(str) / show_len) + 1); i ++)
//                {
//                    for(j = 0; j < show_len; j ++)
//                    {
//                        if(strlen(str) < (j + i * show_len))
//                        {
//                            break;
//                        }
//                        file_path_buffer[j] = str[j + i * show_len];
//                    }

//                    file_path_buffer[j] = '\0';                                 // 鏈熬娣诲姞\0

//                    debug_output_info.output_screen(0, debug_output_info.font_y_size * show_line_index ++, file_path_buffer);
//                }
//            }
//        }
//        else
//        {
//			printf("\r\n %s file %s line %d\r\n", type, file, line);

////            memset(output_buffer, 0, 256);
////            debug_output_info.output_uart(type);
////            if(NULL != str)
////            {
////                sprintf(output_buffer, "\r\nfile %s line %d: %s.\r\n", file, line, str);
////            }
////            else
////            {
////                sprintf(output_buffer, "\r\nfile %s line %d.\r\n", file, line);
////            }
////            debug_output_info.output_uart(output_buffer);
//        }
//    }
//}


//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    璋冭瘯涓插彛鍙戦€佺紦鍐插尯
// 鍙傛暟璇存槑     *buff       璇诲嚭鏁版嵁瀛樻斁鐨勬暟缁勬寚閽?
// 鍙傛暟璇存槑     len         闇€瑕佸彂閫佺殑闀垮害
// 杩斿洖鍙傛暟     uint32      鍓╀綑鏈彂閫佺殑闀垮害
// 浣跨敤绀轰緥
// 澶囨敞淇℃伅     鏈嚱鏁伴渶瑕佸紑鍚?DEBUG_UART_USE_INTERRUPT 瀹忓畾涔夋墠鍙娇鐢?
//-------------------------------------------------------------------------------------------------------------------
uint32 debug_send_buffer(const uint8 *buff, uint32 len)
{
	if(len > 0xFFFF)
	{
		uart_write_buffer(DEBUG_UART_INDEX, buff, 0xFFFF);
		return  len - 0xFFFF;
	}
	else
	{
		uart_write_buffer(DEBUG_UART_INDEX, buff, (uint16)len);
	}
    
    return 0;
}



//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    璇诲彇 debug 鐜舰缂撳啿鍖烘暟鎹?
// 鍙傛暟璇存槑     *buff       璇诲嚭鏁版嵁瀛樻斁鐨勬暟缁勬寚閽?
// 鍙傛暟璇存槑     len         闇€瑕佽鍙栫殑闀垮害
// 杩斿洖鍙傛暟     uint32      璇诲嚭鏁版嵁鐨勫疄闄呴暱搴?
// 浣跨敤绀轰緥
// 澶囨敞淇℃伅     鏈嚱鏁伴渶瑕佸紑鍚?DEBUG_UART_USE_INTERRUPT 瀹忓畾涔夋墠鍙娇鐢?
//-------------------------------------------------------------------------------------------------------------------
uint32 debug_read_buffer (uint8 *buff, uint32 len)
{
    fifo_read_buffer(&debug_uart_fifo, buff, &len, FIFO_READ_AND_CLEAN);

    return len;
}

#if DEBUG_UART_USE_INTERRUPT                                                    // 鏉′欢缂栬瘧 鍙湁鍦ㄥ惎鐢ㄤ覆鍙ｄ腑鏂墠缂栬瘧
//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    debug 涓插彛涓柇澶勭悊鍑芥暟 isr.c 涓搴斾覆鍙ｄ腑鏂湇鍔″嚱鏁拌皟鐢?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     debug_interrupr_handler();
// 澶囨敞淇℃伅     鏈嚱鏁伴渶瑕佸紑鍚?DEBUG_UART_USE_INTERRUPT 瀹忓畾涔夋墠鍙娇鐢?
//              骞朵笖鏈嚱鏁伴粯璁ゆ斁缃湪 UART1 鐨勪覆鍙ｆ帴鏀朵腑鏂鐞嗗
//-------------------------------------------------------------------------------------------------------------------
void debug_interrupr_handler (uint8 dat)
{
	if(zf_debug_init_flag)
	{
		uart_query_byte(DEBUG_UART_INDEX, &dat);                    // 璇诲彇涓插彛鏁版嵁
		fifo_write_buffer(&debug_uart_fifo, &dat, 1);               // 瀛樺叆 FIFO
	}

}

#endif

//-------------------------------------------------------------------------     // printf 閲嶅畾鍚?姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?    printf閲嶅畾鍚?
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
//  @since      v1.0
// 澶囨敞淇℃伅              閲嶅畾鍚憄rintf鍒癉EBUG涓插彛涓?
//-------------------------------------------------------------------------------------------------------------------
#if(1 == PRINTF_ENABLE)      //鍒濆鍖栬皟璇曚覆鍙?
//閲嶅畾涔塸rintf 鏁板瓧 鍙兘杈撳嚭uint16
char putchar(char c)
{
    uart_write_byte(DEBUG_UART_INDEX, c);//鎶婅嚜宸卞疄鐜扮殑涓插彛鎵撳嵃涓€瀛楄妭鏁版嵁鐨勫嚱鏁版浛鎹㈠埌杩欓噷

    return c;
}
#endif
//-------------------------------------------------------------------------     // printf 閲嶅畾鍚?姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?    鍚敤鏂█
//// 鍙傛暟璇存槑     void
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥     debug_assert_enable();
//// 澶囨敞淇℃伅     鏂█榛樿寮€鍚?寤鸿寮€鍚柇瑷€
////-------------------------------------------------------------------------------------------------------------------
//void debug_assert_enable (void)
//{
//    zf_debug_assert_enable = 1;
//}

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?     绂佺敤鏂█
//// 鍙傛暟璇存槑     void
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥     debug_assert_disable();
//// 澶囨敞淇℃伅     鏂█榛樿寮€鍚?涓嶅缓璁鐢ㄦ柇瑷€
////-------------------------------------------------------------------------------------------------------------------
//void debug_assert_disable (void)
//{
//    zf_debug_assert_enable = 0;
//}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?     debug 鏂█澶勭悊鍑芥暟 姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
// 鍙傛暟璇存槑     pass        鍒ゆ柇鏄惁瑙﹀彂鏂█
// 鍙傛暟璇存槑     *file       鏂囦欢鍚?
// 鍙傛暟璇存槑     line        鐩爣琛屾暟
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     zf_assert(0);
// 澶囨敞淇℃伅     杩欎釜鍑芥暟涓嶆槸鐩存帴璋冪敤鐨?姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
//              浣跨敤 zf_commmon_debug.h 涓殑 zf_assert(x) 鎺ュ彛
//-------------------------------------------------------------------------------------------------------------------
void debug_assert_handler (uint8 pass, char *file, int line)
{
	uint8 log_str[] = "Assert error";

	while(!pass)
	{
		// 濡傛灉浠ｇ爜璺宠浆鍒拌繖閲屽仠浣忎簡
		// 涓€鑸綘鐨勫嚱鏁板弬鏁颁紶閫掑嚭閿欎簡
		// 鎴栬€呬綘鑷繁璋冪敤鐨?zf_assert(x) 鎺ュ彛澶勬姤閿欎簡

		// 濡傛灉璋冪敤浜?debug_init 鍒濆鍖栦簡 log 杈撳嚭
		// 灏卞湪瀵瑰簲涓插彛杈撳嚭鍘绘煡鐪嬫槸鍝釜鏂囦欢鐨勫摢涓€琛屾姤閿?

		// 濡傛灉娌℃湁鍒濆鍖?debug
		// 閭ｅ氨鐪嬬湅杩欎釜 file 鐨勫瓧绗︿覆鍊煎拰 line 鐨勮鏁?
		// 閭ｄ唬琛ㄦ姤閿欑殑鏂囦欢璺緞鍚嶇О鍜屽搴旀姤閿欒鏁?

		// 鍐嶅幓璋冭瘯鐪嬬湅鏄负浠€涔堝弬鏁板嚭閿?


		// printf("\r\n %s file %s line %d\r\n", log_str, file, line);


		system_delay_ms(500);
	}
}

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?     debug 璋冭瘯淇℃伅澶勭悊鍑芥暟 姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
//// 鍙傛暟璇存槑     bool        鍒ゆ柇鏄惁瑙﹀彂鏂█
//// 鍙傛暟璇存槑     *str        瑕佽緭鍑虹殑璋冭瘯淇℃伅
//// 鍙傛暟璇存槑     *file       鏂囦欢鍚?
//// 鍙傛暟璇存槑     line        鐩爣琛屾暟
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥     printf( "Log Message");
//// 澶囨敞淇℃伅     杩欎釜鍑芥暟涓嶆槸鐩存帴璋冪敤鐨?姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
////              浣跨敤 zf_commmon_debug.h 涓殑 zf_log(x, str) 鎺ュ彛
////-------------------------------------------------------------------------------------------------------------------
//void debug_log_handler (uint8 pass, char *str, char *file, int line)
//{
//	uint8 log_str[] = "Log message";
//    do
//    {
//        if(pass)
//        {
//            break;
//        }
//        if(zf_debug_init_flag)
//        {
//            debug_output(log_str, file, line, str);
////            printf("Log message from %s line %d :\"%s\".\r\n", file, line, str);
//        }
//    }while(0);
//}

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?     debug 杈撳嚭缁戝畾淇℃伅鍒濆鍖?姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
//// 鍙傛暟璇存槑     *info       debug 杈撳嚭鐨勪俊鎭粨鏋勪綋
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥            debug_output_struct_init(info);
////-------------------------------------------------------------------------------------------------------------------
//void debug_output_struct_init (debug_output_struct *info)
//{
//    info->type_index            = 0;

//    info->display_x_max         = 0xFFFF;
//    info->display_y_max         = 0xFFFF;

//    info->font_x_size           = 0xFF;
//    info->font_y_size           = 0xFF;

//    info->output_uart           = NULL;
//    info->output_screen         = NULL;
//    info->output_screen_clear   = NULL;
//}

////-------------------------------------------------------------------------------------------------------------------
//// 鍑芥暟绠€浠?     debug 杈撳嚭缁戝畾鍒濆鍖?姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
//// 鍙傛暟璇存槑     *info       debug 杈撳嚭鐨勪俊鎭粨鏋勪綋
//// 杩斿洖鍙傛暟     void
//// 浣跨敤绀轰緥     debug_output_init(info);
//// 澶囨敞淇℃伅     杩欎釜鍑芥暟涓€鑸笉鐢辩敤鎴疯皟鐢?
////-------------------------------------------------------------------------------------------------------------------
//void debug_output_init (debug_output_struct *info)
//{
//    debug_output_info.type_index            = info->type_index;

//    debug_output_info.display_x_max         = info->display_x_max;
//    debug_output_info.display_y_max         = info->display_y_max;

//    debug_output_info.font_x_size           = info->font_x_size;
//    debug_output_info.font_y_size           = info->font_y_size;

//    debug_output_info.output_uart           = info->output_uart;
//    debug_output_info.output_screen         = info->output_screen;
//    debug_output_info.output_screen_clear   = info->output_screen_clear;

//    zf_debug_init_flag = 1;
//}

//-------------------------------------------------------------------------------------------------------------------
// 鍑芥暟绠€浠?     debug 涓插彛鍒濆鍖?姝ら儴鍒嗕笉鍏佽鐢ㄦ埛鏇存敼
// 鍙傛暟璇存槑     void
// 杩斿洖鍙傛暟     void
// 浣跨敤绀轰緥     debug_init();
// 澶囨敞淇℃伅     寮€婧愬簱绀轰緥榛樿璋冪敤 浣嗛粯璁ょ鐢ㄤ腑鏂帴鏀?
//-------------------------------------------------------------------------------------------------------------------
void debug_init (void)
{
	uint8 uartx = DEBUG_UART_INDEX;
//    debug_output_struct info;
//    debug_output_struct_init(&info);
//    info.output_uart = debug_uart_str_output;
//    debug_output_init(&info);

    uart_init(
        DEBUG_UART_INDEX,                                                       // 鍦?zf_common_debug.h 涓煡鐪嬪搴斿€?
        DEBUG_UART_BAUDRATE,                                                    // 鍦?zf_common_debug.h 涓煡鐪嬪搴斿€?
        DEBUG_UART_TX_PIN,                                                      // 鍦?zf_common_debug.h 涓煡鐪嬪搴斿€?
        DEBUG_UART_RX_PIN);                                                     // 鍦?zf_common_debug.h 涓煡鐪嬪搴斿€?

#if DEBUG_UART_USE_INTERRUPT                                                    // 鏉′欢缂栬瘧 鍙湁鍦ㄥ惎鐢ㄤ覆鍙ｄ腑鏂墠缂栬瘧
    fifo_init(&debug_uart_fifo, FIFO_DATA_8BIT, debug_uart_buffer, DEBUG_RING_BUFFER_LEN);
    uart_rx_interrupt(DEBUG_UART_INDEX, 1);                                     // 浣胯兘瀵瑰簲涓插彛鎺ユ敹涓柇

	// 璁剧疆涓插彛鍥炶皟鍑芥暟
	if(uartx == UART_1)				
    {
        uart1_irq_handler = debug_interrupr_handler;
    }
    else if(uartx == UART_2)
    {
        uart2_irq_handler = debug_interrupr_handler;
    }
    else if(uartx == UART_3)
    {
        uart3_irq_handler = debug_interrupr_handler;
    }
    else if(uartx == UART_4)
    {
        uart4_irq_handler = debug_interrupr_handler;
    }

#endif

}




