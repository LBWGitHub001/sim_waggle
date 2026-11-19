#ifndef COMMON__CHANNEL_HPP
#define COMMON__CHANNEL_HPP

#include <condition_variable>
#include <mutex>
#include <queue>

namespace common
{

/// 线程安全 FIFO 消息通道（类似 ROS topic 的接收队列）
/// 多生产者-单消费者语义：push() 永不阻塞，wait_pop() 阻塞等待。
/// close() 后 wait_pop() 把队列中剩余消息排空才返回 false，
/// 保证每条消息都被消费，收尾行为确定。
template <typename T>
class Channel
{
public:
  // 投递一条消息（线程安全，永不阻塞）
  void push(T msg)
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      if (closed_) return;
      q_.push(std::move(msg));
    }
    cv_.notify_one();
  }

  // 阻塞取下一条消息；返回 false 表示通道已关闭且队列已排空
  bool wait_pop(T & out)
  {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&] { return closed_ || !q_.empty(); });
    if (q_.empty()) return false;
    out = std::move(q_.front());
    q_.pop();
    return true;
  }

  // 关闭通道：唤醒所有等待者，剩余消息仍可被取走
  void close()
  {
    {
      std::lock_guard<std::mutex> lk(m_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  // 当前积压深度（调试指标：持续 >1 说明消费者跟不上生产者）
  size_t size() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return q_.size();
  }

private:
  mutable std::mutex m_;
  std::condition_variable cv_;
  std::queue<T> q_;
  bool closed_ = false;
};

/// 线程安全"最新值"槽（类似 ROS topic keep_last(1) 的快照读法）
/// 写方覆盖式更新，读方取走一份完整拷贝；用于可视化侧读数。
template <typename T>
class LatestSlot
{
public:
  void set(T v)
  {
    std::lock_guard<std::mutex> lk(m_);
    value_ = std::move(v);
    has_value_ = true;
  }

  // 有值时拷贝一份到 out 并返回 true；从未写入过返回 false
  bool get(T & out) const
  {
    std::lock_guard<std::mutex> lk(m_);
    if (!has_value_) return false;
    out = value_;
    return true;
  }

private:
  mutable std::mutex m_;
  T value_{};
  bool has_value_ = false;
};

}  // namespace common

#endif  // COMMON__CHANNEL_HPP
