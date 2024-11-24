/**
 * @file pid_template.h
 * @author Elaina (1463967532@qq.com)
 * @brief pid的参数模版库
 * 使用方法1.定义: pid_base_template_t<int16t,float>pid({0.1,0,0,-60,60,2000});
 * 或者: pid_base_template_t<int16t,float>pid;PidBaseConfig_T<int16t,float>config={0.1,0,0,-60,60,2000};pid(config);
 * 调用方法2.pid.target_update(target)先设置目标值 然后再循环中调用pid.update(contrl)更新控制值
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __PID_TEMPLATE_H
#define __PID_TEMPLATE_H
/**
 * @brief pid的配置结构体分别有kp,ki,kd,out_min输出下限,out_max输出上限,integral_max积分极大值
 * 
 * @tparam T  目标值和控制值的数据类型
 * @tparam T2 pid参数的数据类型
 */
template <typename T, typename T2>
struct PidBaseConfig_T
{
    T2 kp = 0;
    T2 ki = 0;
    T2 kd = 0;
    T out_min = 0;
    T out_max = 0;
    T integral_max = 25;
};

// 位置式pid
template <typename T, typename T2>
class pid_base_template_t
{
public:
    /**
     * @brief Construct a new pid base template t object
     * 
     */
    pid_base_template_t() {}; // 默认构造函数
    /**
     * @brief 通过配置结构体配置pid
     * 
     * @param config 结构体
     */
    pid_base_template_t(PidBaseConfig_T<T, T2> config) : kp_(config.kp), ki_(config.ki), kd_(config.kd), out_min_(config.out_min), out_max_(config.out_max)
    {
        error_sum_max = config.integral_max;
    };

    T target_;
    T2 kp_;         // 比例系数
    T2 ki_;         // 积分系数
    T2 kd_;         // 微分系数
    T last_output_; // 上一次输出值
    /**
     * @brief 需要预设target的pid更新
     * 
     * @param contrl 实际值
     * @return T 控制量
     */
    T update(T contrl) 
    { 
        error = target_ - contrl;
        error_sum += error;
        error_delta = error_last - error;
        error_last = error;

        if (error_sum > error_sum_max)
        {
            error_sum = error_sum_max;
        }
        if (error_sum < -error_sum_max)
        {
            error_sum = -error_sum_max;
        }

        T output = kp_ * error + ki_ * error_sum + kd_ * error_delta;
        if (output > out_max_)
        {
            output = out_max_;
        }
        else if (output < out_min_)
        {
            output = out_min_;
        }

        last_output_ = output;
        return output;
    }
    /**
     * @brief 不用设置target的pid更新
     * 
     * @param target 目标值
     * @param contrl 当前值
     * @param clear_integral 是否清除积分,默认不清除 
     * @return T 控制量
     */
    T cal(T target, T contrl, bool clear_integral = false) 
    { 
        target_update(target, clear_integral);
        return update(contrl); 
    }
    /**
     * @brief pid重置,把所有参数都清零
     * 
     */
    void reset(void) 
    {
        last_output_ = 0.0f; // 上一次的控制输出值
        target_ = 0.0f;      // 控制目标值
        kp_ = 0.0;
        ki_ = 0.0;
        kd_ = 0.0;

        error = 0.0;
        error_delta = 0.0;
        error_last = 0.0;
        error_sum = 0.0;
        error_pre = 0.0;
    }
    /**
     * @brief 设置目标值
     * 
     * @param target 目标值
     * @param clear_integral 是否清除积分,默认不清除
     */
    void target_update(T target, bool clear_integral = false)
    {
        target_ = target;
        if (clear_integral)
        {
            error_sum = 0;
        }
    }
    /**
     * @brief 设置pid参数，已废弃不要使用
     * 
     * @param kp 
     * @param ki 
     * @param kd 
     */
    void pid_update(T2 kp, T2 ki, T2 kd) 
    { 
        reset();
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    /**
     * @brief 输出限幅配置,已经弃用
     * 
     * @param out_min 
     * @param out_max 
     */
    void out_limit(T out_min, T out_max) 
    {
        out_min_ = out_min;
        out_max_ = out_max;
    }
    /**
     * @brief 积分清除
     * 
     */
    void reset_integral(void) 
    {
        error_sum = 0;
    }
    /**
     * @brief 输出限幅
     * 
     * @param output pid原始输出
     * @return T 限幅输出
     */
    T output_limit(T output) 
    {
        if (output > out_max_)
        {
            output = out_max_;
        }
        else if (output < out_min_)
        {
            output = out_min_;
        }
        return output;
    }

protected:
    T error;              // 误差
    T error_sum;          // 累计误差
    T error_sum_max = 25; // 积分上限;

    T error_delta; // 误差微分
    T error_last;  // 上一次的误差

    T error_pre; // 前次的误差

    T out_min_; // 输出下限
    T out_max_; // 输出上限
};

// 前馈pid
template <typename T, typename T2>
class pid_foward_template_t : public pid_base_template_t<T, T2>
{
public:
    pid_foward_template_t(PidBaseConfig_T<T, T2> config, T forward_k = 0) : pid_base_template_t<T, T2>(config), forwardfeed_k_(forward_k) {};

    T update(T contrl, bool clear_integral = false) 
    { 
        return this->output_limit(pid_base_template_t<T, T2>::update(contrl) + forwardfeed()); 
    }

    T forwardfeed() 
    { 
        return forwardfeed_k_ * this->target_; 
    }

    T cal(T target, T contrl, bool clear_integral = false) 
    { 
        this->target_update(target, clear_integral); 
        return update(contrl); 
    }

private:
    T2 forwardfeed_k_ = 0;
};

// 增量式pid
template <typename T, typename T2>
class pid_Increment_template_t : public pid_base_template_t<T, T2>
{
public:
    pid_Increment_template_t(PidBaseConfig_T<T, T2> config) : pid_base_template_t<T, T2>(config) {};

    T update(T contrl) 
    { 
        this->error = this->target_ - contrl; 
        T output = this->kp_ * this->error + this->ki_ * this->error_last + this->kd_ * (this->error - 2 * this->error_last + this->error_last_last);

        this->error_last_last = this->error_last;
        this->error_last = this->error;

        if (output > this->out_max_)
        {
            output = this->out_max_;
        }
        else if (output < this->out_min_)
        {
            output = this->out_min_;
        }
        return output;
    }

    T cal(T target, T contrl, bool clear_integral = false) 
    { 
        this->target_update(target, clear_integral); 
        return update(contrl); 
    }

private:
    T error_last_last;
};

#endif
