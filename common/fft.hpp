#ifndef COMMON__FFT_HPP
#define COMMON__FFT_HPP

#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

namespace common
{

/// 迭代 radix-2 Cooley-Tukey FFT（就地计算，输入长度必须为 2 的幂）
inline void radix2_fft(std::vector<std::complex<double>> & a)
{
  const size_t n = a.size();
  // 位反转重排
  for (size_t i = 1, j = 0; i < n; i++) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(a[i], a[j]);
  }
  // 蝶形运算
  for (size_t len = 2; len <= n; len <<= 1) {
    const double ang = -2.0 * M_PI / static_cast<double>(len);
    const std::complex<double> wlen(std::cos(ang), std::sin(ang));
    for (size_t i = 0; i < n; i += len) {
      std::complex<double> w(1.0);
      for (size_t j = 0; j < len / 2; j++) {
        const std::complex<double> u = a[i + j];
        const std::complex<double> v = a[i + j + len / 2] * w;
        a[i + j] = u + v;
        a[i + j + len / 2] = u - v;
        w *= wlen;
      }
    }
  }
}

/// 滑窗 FFT 分析器 (FftAnalyzer)
/// 喂入时间序列采样点，攒满 N（2 的幂）点后每 hop 个样本输出一次单边幅度谱。
/// Hann 窗抑制频谱泄漏；采样率由时间戳估计（fs = (N-1)/(t_last - t_first)），
/// 幅度按 Hann 窗相干增益归一化（正弦幅值可直接读数）。
class FftAnalyzer
{
public:
  // n: 窗口长度（自动向下取整为 2 的幂，最小 8）；hop: 每多少个样本输出一次
  explicit FftAnalyzer(size_t n = 512, size_t hop = 0) : n_(n < 8 ? 8 : n), hop_(hop)
  {
    // 向下取整为 2 的幂
    while (n_ & (n_ - 1)) n_ &= n_ - 1;
    if (hop_ == 0 || hop_ > n_) hop_ = n_ / 2;
  }

  // 喂入一个采样点（v: 值, t: 时间戳，单位秒）；窗口就绪可 compute() 时返回 true
  bool push(double v, double t)
  {
    buf_.push_back(v);
    ts_.push_back(t);
    if (buf_.size() > n_) {
      buf_.erase(buf_.begin());
      ts_.erase(ts_.begin());
    }
    if (buf_.size() < n_) return false;
    if (++since_last_ < hop_) return false;
    since_last_ = 0;
    return true;
  }

  // 计算当前窗口的单边幅度谱：
  //   freq[i] = i * fs / N（i = 0..N/2），mag 为对应幅度，fs 为估计采样率 (Hz)
  void compute(std::vector<double> & freq, std::vector<double> & mag, double & fs) const
  {
    freq.clear();
    mag.clear();
    fs = 0.0;
    if (buf_.size() < n_) return;

    const double span = ts_.back() - ts_.front();
    fs = span > 1e-9 ? static_cast<double>(n_ - 1) / span : 0.0;

    // 去均值（去 DC）：避免大直流分量经窗泄漏淹没低频段谱峰
    double mean = 0.0;
    for (double v : buf_) mean += v;
    mean /= static_cast<double>(n_);

    // Hann 窗
    std::vector<std::complex<double>> a(n_);
    double w_sum = 0.0;
    for (size_t i = 0; i < n_; i++) {
      const double w =
        0.5 * (1.0 - std::cos(2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n_ - 1)));
      a[i] = (buf_[i] - mean) * w;
      w_sum += w;
    }
    radix2_fft(a);

    // 单边幅度谱（按窗函数相干增益归一化：正弦峰值 = 真实幅值）
    const size_t half = n_ / 2;
    const double df = fs / static_cast<double>(n_);
    freq.resize(half + 1);
    mag.resize(half + 1);
    for (size_t i = 0; i <= half; i++) {
      freq[i] = static_cast<double>(i) * df;
      mag[i] = std::abs(a[i]) * (i == 0 ? 1.0 : 2.0) / w_sum;
    }
  }

  size_t window_size() const { return n_; }

private:
  size_t n_;
  size_t hop_;
  size_t since_last_ = 0;  // 距上次就绪的样本数
  std::vector<double> buf_;
  std::vector<double> ts_;
};

}  // namespace common

#endif  // COMMON__FFT_HPP
