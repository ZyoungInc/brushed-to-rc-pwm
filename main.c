#include "REG52.H"

/* =========================================================
   STC8G1K08A-8PIN
   35MHz / Timer0 1T / 10us tick
   输出：100Hz RC 脉冲

   输入：
       A -> P3.0 (pin 5)
       B -> P3.1 (pin 6)
       C -> P3.2 (pin 7)
       D -> P3.3 (pin 8)

   输出：
       E -> P5.4 (pin 1)
       F -> P5.5 (pin 3)

   输入实际情况：
       只有 0, 10%, 20%, 30%, 40%, 50%, 60% 这几挡
       PWM 频率约 314Hz
       A/B 不会同时有效，C/D 不会同时有效

   本版本策略：
       1) 先测一帧(10ms)内的输入高电平占比
       2) 量化到 0/10/20/30/40/50/60
       3) 加“档位容错/迟滞”：
          当前在 10% 档时，只要输入还在 5%~15%，就继续保持 10% 档
       4) 用查表映射成 RC 脉宽
   ========================================================= */


/* =========================
   参数区
   ========================= */
#define FOSC                    35000000UL     /* 35MHz */
#define TICK_US                 10UL           /* 中断节拍 10us，稳定优先 */
#define FRAME_US                20000UL        /* 10ms = 100Hz */
#define MID_US                  1500UL         /* 中位 */
#define ARM_TIME_MS             2000UL         /* 上电先保持中位 2秒 */

/* 输入量化相关 */
#define INPUT_STOP_PERCENT_X10  50UL           /* 小于 5.0% 当作停 */
#define INPUT_MAX_PERCENT_X10   600UL          /* 最大只认到 60.0% */

/* 档位容错：一个 10% 档，允许 ±5% 浮动 */
#define STEP_HALF_BAND_X10      50UL           /* 5.0% */

/* 由上面参数自动算出 */
#define FRAME_TICKS             (FRAME_US / TICK_US)   /* 1000 */
#define MID_TICKS               (MID_US   / TICK_US)   /* 150 */
#define ARM_FRAMES              ((ARM_TIME_MS * 1000UL) / FRAME_US)

/* Timer0 重装值：35MHz, 1T, 10us */
#define TIMER0_RELOAD           (65536UL - (FOSC / 1000000UL * TICK_US))
#define TIMER0_RH               ((unsigned char)(TIMER0_RELOAD >> 8))
#define TIMER0_RL               ((unsigned char)(TIMER0_RELOAD & 0xFF))

/* =========================
   引脚定义
   ========================= */
sbit IN_A  = P3^0;   /* pin 5 */
sbit IN_B  = P3^1;   /* pin 6 */
sbit IN_C  = P3^2;   /* pin 7 */
sbit IN_D  = P3^3;   /* pin 8 */

sbit OUT_E = P5^4;   /* pin 1 */
sbit OUT_F = P5^5;   /* pin 3 */

/* =========================
   用户可调输出表
   下标含义：
     [0] = 0%
     [1] = 10%
     [2] = 20%
     [3] = 30%
     [4] = 40%
     [5] = 50%
     [6] = 60%
   ========================= */
const unsigned int code pos_map_us[7] =
{
    1500,   /* 0%  */
    1545,   /* 10% */
    1595,   /* 20% */
    1665,   /* 30% */
    1760,   /* 40% */
    1875,   /* 50% */
    2000    /* 60% */
};

const unsigned int code neg_map_us[7] =
{
    1500,   /* 0%  */
    1455,   /* 10% */
    1405,   /* 20% */
    1335,   /* 30% */
    1240,   /* 40% */
    1125,   /* 50% */
    1000    /* 60% */
};

/* =========================
   全局变量
   ========================= */
volatile unsigned int g_tick_in_frame = 0;

volatile unsigned int g_pulse_e_ticks = MID_TICKS;
volatile unsigned int g_pulse_f_ticks = MID_TICKS;

volatile unsigned int g_cnt_a = 0;
volatile unsigned int g_cnt_b = 0;
volatile unsigned int g_cnt_c = 0;
volatile unsigned int g_cnt_d = 0;

volatile unsigned int g_arm_frames = ARM_FRAMES;

/* 记住上一次档位
   0  = 停
   1~6 = 正向 10%~60%
  -1~-6 = 反向 10%~60%
*/
volatile signed char g_last_step_e = 0;
volatile signed char g_last_step_f = 0;

/* =========================
   GPIO初始化
   P3.0~P3.3：输入
   P5.4~P5.5：推挽输出
   ========================= */
void gpio_init(void)
{
    /* P3.0~P3.3 高阻输入：M1=1, M0=0 */
    P3M1 |=  0x0F;
    P3M0 &= ~0x0F;

    /* P5.4~P5.5 推挽输出：M1=0, M0=1 */
    P5M1 &= ~0x30;
    P5M0 |=  0x30;

    OUT_E = 0;
    OUT_F = 0;
}

/* =========================
   us -> ticks
   ========================= */
static unsigned int us_to_ticks(unsigned int us)
{
    return (unsigned int)((us + (TICK_US / 2)) / TICK_US);
}

/* =========================
   cnt -> 0.1%单位
   10.0% -> 100
   60.0% -> 600
   ========================= */
static unsigned int count_to_pct_x10(unsigned int cnt)
{
    return (unsigned int)(((unsigned long)cnt * 1000UL + (FRAME_TICKS / 2)) / FRAME_TICKS);
}

/* =========================
   无记忆量化：四舍五入到最近的 10% 挡
   返回 0~6
   ========================= */
static unsigned char quantize_step(unsigned int pct_x10)
{
    unsigned char step;

    if (pct_x10 < INPUT_STOP_PERCENT_X10)
    {
        return 0;
    }

    if (pct_x10 > INPUT_MAX_PERCENT_X10)
    {
        pct_x10 = INPUT_MAX_PERCENT_X10;
    }

    step = (unsigned char)((pct_x10 + 50UL) / 100UL);

    if (step > 6)
    {
        step = 6;
    }

    return step;
}

/* =========================
   带容错/迟滞的档位更新
   last_step:
      0  = 停
      1~6 = 正向
     -1~-6 = 反向
   positive:
      1 = 正向输入
      0 = 反向输入
   返回：
      0 / ±1..±6
   ========================= */
static signed char update_step_with_hysteresis(unsigned int pct_x10, signed char last_step, bit positive)
{
    unsigned int low_bound, high_bound;
    unsigned char abs_step;
    unsigned char new_step;

    /* 小于停止阈值 */
    if (pct_x10 < INPUT_STOP_PERCENT_X10)
    {
        return 0;
    }

    if (pct_x10 > INPUT_MAX_PERCENT_X10)
    {
        pct_x10 = INPUT_MAX_PERCENT_X10;
    }

    /* 如果上一次方向一致，则先判断是否还在“当前档 ±5%”的容错区间 */
    if (positive)
    {
        if (last_step > 0)
        {
            abs_step = (unsigned char)last_step;   /* 1~6 */
            low_bound  = (unsigned int)abs_step * 100UL - STEP_HALF_BAND_X10;
            high_bound = (unsigned int)abs_step * 100UL + STEP_HALF_BAND_X10;

            if ((pct_x10 >= low_bound) && (pct_x10 <= high_bound))
            {
                return last_step;
            }
        }
    }
    else
    {
        if (last_step < 0)
        {
            abs_step = (unsigned char)(-last_step);  /* 1~6 */
            low_bound  = (unsigned int)abs_step * 100UL - STEP_HALF_BAND_X10;
            high_bound = (unsigned int)abs_step * 100UL + STEP_HALF_BAND_X10;

            if ((pct_x10 >= low_bound) && (pct_x10 <= high_bound))
            {
                return last_step;
            }
        }
    }

    /* 超出当前档容错区，才重新量化 */
    new_step = quantize_step(pct_x10);

    if (new_step == 0)
    {
        return 0;
    }

    if (positive)
    {
        return (signed char)new_step;
    }
    else
    {
        return (signed char)(-((signed char)new_step));
    }
}

/* =========================
   由 signed step -> pulse ticks
   ========================= */
static unsigned int step_to_ticks(signed char step)
{
    if (step == 0)
    {
        return MID_TICKS;
    }

    if (step > 0)
    {
        return us_to_ticks(pos_map_us[(unsigned char)step]);
    }

    return us_to_ticks(neg_map_us[(unsigned char)(-step)]);
}

/* =========================
   根据一对输入，计算输出档位
   正向看 pos_map_us[]
   反向看 neg_map_us[]
   ========================= */
static signed char calc_step_from_pair(unsigned int cnt_pos, unsigned int cnt_neg, signed char last_step)
{
    unsigned int pct_pos, pct_neg;

    pct_pos = count_to_pct_x10(cnt_pos);
    pct_neg = count_to_pct_x10(cnt_neg);

    /* 都没有 -> 中位 */
    if ((pct_pos < INPUT_STOP_PERCENT_X10) && (pct_neg < INPUT_STOP_PERCENT_X10))
    {
        return 0;
    }

    /* 同时有效 -> 异常，回中 */
    if ((pct_pos >= INPUT_STOP_PERCENT_X10) && (pct_neg >= INPUT_STOP_PERCENT_X10))
    {
        return 0;
    }

    /* 正向 */
    if (pct_pos >= INPUT_STOP_PERCENT_X10)
    {
        return update_step_with_hysteresis(pct_pos, last_step, 1);
    }

    /* 反向 */
    return update_step_with_hysteresis(pct_neg, last_step, 0);
}

/* =========================
   Timer0初始化
   10us中断一次
   ========================= */
void timer0_init(void)
{
    AUXR |= 0x80;     /* Timer0 1T 模式 */

    TMOD &= 0xF0;
    TMOD |= 0x01;     /* 16位定时器 */

    TH0 = TIMER0_RH;
    TL0 = TIMER0_RL;

    ET0 = 1;
    EA  = 1;
    TR0 = 1;
}

/* =========================
   Timer0中断
   1) 采样 A/B/C/D 占空比
   2) 输出 E/F RC脉冲
   ========================= */
void timer0_isr(void) interrupt 1 using 1
{
    TH0 = TIMER0_RH;
    TL0 = TIMER0_RL;

    /* 采样输入高电平时间 */
    if (IN_A) g_cnt_a++;
    if (IN_B) g_cnt_b++;
    if (IN_C) g_cnt_c++;
    if (IN_D) g_cnt_d++;

    /* 每帧开始拉高输出 */
    if (g_tick_in_frame == 0)
    {
        OUT_E = 1;
        OUT_F = 1;
    }

    /* 到达目标脉宽后拉低 */
    if (g_tick_in_frame == g_pulse_e_ticks)
    {
        OUT_E = 0;
    }

    if (g_tick_in_frame == g_pulse_f_ticks)
    {
        OUT_F = 0;
    }

    g_tick_in_frame++;

    /* 一帧结束 */
    if (g_tick_in_frame >= FRAME_TICKS)
    {
        g_tick_in_frame = 0;

        /* 上电先保持中位 2秒 */
        if (g_arm_frames > 0)
        {
            g_arm_frames--;
            g_last_step_e = 0;
            g_last_step_f = 0;
            g_pulse_e_ticks = MID_TICKS;
            g_pulse_f_ticks = MID_TICKS;
        }
        else
        {
            g_last_step_e = calc_step_from_pair(g_cnt_a, g_cnt_b, g_last_step_e);
            g_last_step_f = calc_step_from_pair(g_cnt_c, g_cnt_d, g_last_step_f);

            g_pulse_e_ticks = step_to_ticks(g_last_step_e);
            g_pulse_f_ticks = step_to_ticks(g_last_step_f);
        }

        g_cnt_a = 0;
        g_cnt_b = 0;
        g_cnt_c = 0;
        g_cnt_d = 0;
    }
}

/* =========================
   主函数
   ========================= */
void main(void)
{
    gpio_init();
    timer0_init();

    while (1)
    {
    }
}