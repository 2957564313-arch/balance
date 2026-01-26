#ifndef __KEIL_INTRINS_STUB_H__
#define __KEIL_INTRINS_STUB_H__

#ifdef __cplusplus
extern "C" {
#endif

// 仅用于 VS Code IntelliSense，不参与 Keil 真编译
static __inline void _nop_(void)
{
#if defined(__GNUC__)
    __asm__ __volatile__("nop");
#endif
}

#ifdef __cplusplus
}
#endif

#endif
