/*
 *    archos-touchscreen-cyttsp4-9.h : 12/06/2012
 *    g.revaillot, revaillot@archos.com
 */

#ifndef _ARCHOS_TOUCHSCREEN_CYTTSP4
#define _ARCHOS_TOUCHSCREEN_CYTTSP4

#define CY_I2C_NAME     "cyttsp4-i2c"

#define CY_USE_TMA400_SP2
//#define CY_USE_TMA400
#define CY_USE_TMA884

#ifdef CY_USE_TMA400
#define CY_I2C_TCH_ADR	0x24
#define CY_I2C_LDR_ADR	0x24
#ifdef CY_USE_BUTTON_TEST_PANEL
#define CY_MAXX 600
#define CY_MAXY 800
#else
#define CY_MAXX 880
#define CY_MAXY 1280
#endif
#define CY_MINX 0
#define CY_MINY 0
#endif /* --CY_USE_TMA400 */

#ifdef CY_USE_TMA884
#define CY_I2C_TCH_ADR	0x67
#define CY_I2C_LDR_ADR	0x69
#define CY_MAXX 1280
#define CY_MAXY 480
#define CY_MINX 0
#define CY_MINY 0
#endif /* --CY_USE_TMA884 */

#define CY_ABS_MIN_X CY_MINX
#define CY_ABS_MIN_Y CY_MINY
#define CY_ABS_MIN_P 0
#define CY_ABS_MIN_W 0
#ifdef CY_USE_TMA400
#define CY_ABS_MIN_T 0
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
#define CY_ABS_MIN_T 1
#endif /* --CY_USE_TMA884 */

#define CY_ABS_MAX_X CY_MAXX
#define CY_ABS_MAX_Y CY_MAXY
#define CY_ABS_MAX_P 255
#define CY_ABS_MAX_W 255
#ifdef CY_USE_TMA400
#define CY_ABS_MAX_T 15
#endif /* --CY_USE_TMA400 */
#ifdef CY_USE_TMA884
#define CY_ABS_MAX_T 10
#endif /* --CY_USE_TMA884 */
#define CY_IGNORE_VALUE 0xFFFF

void archos_touchscreen_cyttsp4_init(struct touch_platform_data *pdata);

#endif
