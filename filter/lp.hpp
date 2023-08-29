#ifndef __WWCONTROL_LP_HPP__
#define __WWCONTROL_LP_HPP__

#include "base.hpp"
namespace wibot::control {
struct FirstOrderLowPassFilterConfig {
    /**
     * 采样间隔
     */
    float sample_time;
    /**
     * 截止频率
     */
    float cutoff_freq;
    /**
     * 启用数值折叠。用于周期循环变化的值，比如旋转角度的弧度。
     */
    bool  enable_wrap = false;
    /**
     * 折叠值。比如用于旋转角度弧度时，为2pi。
     */
    float wrap_value;
};
// template<typename T>
class FirstOrderLowPassFilter : public Configurable<FirstOrderLowPassFilterConfig> {
   public:
    Result apply_config() override;

    float filter(float input);
    void  reset(float initValue = 0.0f);

   private:
    float _alpha;
    float _1_alpha;

    float _outputLast;
};

// using FirstOrderLowPassFilterf = FirstOrderLowPassFilter<float>;
// using FirstOrderLowPassFilters = FirstOrderLowPassFilter<uint16_t>;
// using FirstOrderLowPassFilterl = FirstOrderLowPassFilter<uint32_t>;
}  // namespace wibot::control

#endif  // __WWCONTROL_LP_HPP__
