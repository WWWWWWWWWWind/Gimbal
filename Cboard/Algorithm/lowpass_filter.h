#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__
#ifdef __cplusplus
extern "C" {  // 仅C++编译器处理此段
#endif

    // 定义一阶低通滤波器结构体
    typedef struct {
        float alpha;           // 时间常数
        float previous_output; // 上一时刻的输出
    } LowPassFilter;

    extern LowPassFilter Filter;
    extern float filterValue(LowPassFilter* filter, float input);
    extern void initializeFilter(LowPassFilter* filter, float alpha);

#ifdef __cplusplus
}
#endif

#endif
