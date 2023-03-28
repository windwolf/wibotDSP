//
// Created by zhouj on 2023/3/27.
//

#ifndef AQ_DJ_2022_LIBS_WIBOTDSP_FILTER_HYSTERESISCOMPARER_HPP_
#define AQ_DJ_2022_LIBS_WIBOTDSP_FILTER_HYSTERESISCOMPARER_HPP_

#include <cstdint>
namespace wibot::control
{
    template<typename T>
    struct HysteresisComparerBounds
    {
        T low_bound;
        T high_bound;
    };

    /**
     * @brief 多阈值迟滞比较器。
     * N组由上下界组成的阈值对，将区间划分成了2N+1段，其中包括N段滞回区，和N+1段正常区间。当值落在阈值的滞回区时，返回上一次的结果，否则返回该阈值对的索引，其中0表示最低阈值以下，N表示最高阈值的以上。
     * @tparam T
     */
    template<typename T>
    class HysteresisComparer
    {
     public:
        /**
         *
         * @param bounds bounds 阈值对必须从低到高排列，且各阈值的滞回区不得重叠。
         * @param bounds_count
         */
        HysteresisComparer(const HysteresisComparerBounds<T> bounds[], uint32_t bounds_count)
            : bounds_(bounds), bounds_count_(bounds_count), last_result_(0)
        {
        }

        /**
         * @brief 当值处于阈值的滞回区时，返回上一次的结果；否则返回该阈值对的索引，其中0表示最低阈值以下，N表示最高阈值的以上。
         * @param value
         * @return
         */
        uint32_t compare(T value)
        {
            for (int i = 0; i < bounds_count_; ++i)
            {
                if (value < bounds_[i].low_bound)
                {
                    last_result_ = i;
                    return last_result_;
                }
                else if (value >= bounds_[i].low_bound && value < bounds_[i].high_bound)
                {
                    if (last_result_ <= i)
                    {
                        last_result_ = i;
                    }
                    else
                    {
                        last_result_ = i + 1;
                    }
                    return last_result_;
                }
            }
            last_result_ = bounds_count_;
            return last_result_;
        }

     private:
        const HysteresisComparerBounds<T>* bounds_;
        uint32_t bounds_count_;
        uint32_t last_result_;

    };

} // wibot

#endif //AQ_DJ_2022_LIBS_WIBOTDSP_FILTER_HYSTERESISCOMPARER_HPP_
