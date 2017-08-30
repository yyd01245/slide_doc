# 交互式音视频 QoS 保障技术

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }
  .reveal h1{
    text-align: center;
    font-size: 45px;
  }  
</style>



* Qos
* Rtp/Rtcp
* 抗丢包
* 网络拥塞控制
* 关键帧请求
* Jitter buffer


---

## Qos 概念

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
  .reveal p{
    text-align: left;
    font-size: 18px;
  }    
</style>

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/qos_qoe.png)


影响实时音视频质量的主要因素包括：带宽、延时、抖动、丢包。

Qos（Quality of Service）指一个网络能够利用各种基础技术，为指定的网络通讯提供更好的服务能力, 是网络的一种安全机制， 是用来解决网络延迟和阻塞等问题的一种技术。所以当网络过载或拥塞时，Qos 能确保重要业务量不受延迟或丢弃，提高音视频质量。

---

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>

无 Qos 机制的流媒体传输方案

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/no_qos.png)

---


## Rtp/Rtcp 

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>

交互式实时视频应用通常采用 RTP 协议进行音视频传输，RTP头部提供了诸如负载类型、时间戳、序列号和同步源等信息保证基本的音视频实时传输需求。RTP 协议底层采用不可靠的 UDP 传输层协议，当网络过载或拥塞，无法实现对丢包、抖动、乱序以及网络拥塞的自适应调整。RTP/RTCP 是 WebRTC 的重要基础。RFC3550／3551 定义 RTP/RTCP 协议的基本内容，包括报文格式、传输规则等。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/rtp_header.png)

---

RTCP 周期性发送控制包信息.

* SR：发送者报告，描述作为活跃发送者成员的发送和接收统计数字.
* RR：接收者报告，描述非活跃发送者成员的接收统计数字.

---


## 抗丢包

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>


* 丢包重传(ARQ)
* 前向纠错(FEC)
* 丢包隐藏(PLC)

---

### 丢包重传(ARQ)

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>


否定应答(NACK): 发送者通过接受者的反馈得知有报文在传输过程中有丢失，重传该报文。

基于 NACK 反馈的丢包重传方法(RFC4585)：接收端循环检查接收缓冲，当发现丢包后使用 RTCP NACK 反馈报文将丢包信息反馈给发送端；发送端接收NACK反馈并解析后从发送缓存取出对应 RTP 包，并再次发送给接收端。该方法的缺点是增大了端到端的延迟，尤其在丢包大量发生时更为明显。

---

### 前向纠错(FEC)

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>


![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/fec.jpg)在编码发送端发送冗余数据，解码端根据该冗余数据，对丢失的码流进行恢复

FEC 机制是在接收端根据视频帧的重要性（参考帧或非参考帧）发送冗余的视频 RTP 包，在接收端如果检测到丢包则利用冗余包进行恢复，否则将冗余包丢弃。该方法的优点是视频延迟低，但发送冗余包占用了额外的带宽资源。

---

### 丢包隐藏(PLC)<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>
Packet Loss Concealment, 音频数据包丢失时会导致语音失真，为了减轻信包丢失对语音感知质量的影响，因为在声音信号特别是语音信号中存在大量的短期自相似性。PLC 算法利用丢失信包的前一信包或邻接信包（在后一信包可获得的情况下）预测丢失的数据包，尽可能地恢复出原来的语音信息。PLC 算法多数基于接收端处理，不需要发送端参与。属于后向纠错。丢包隐藏技术这类技术适用于相对小的丢包率( 有研究者说是小于 15%)和 小 长 度 的 包(4~40ms).---Opus PLC 模块```
int WebRtcOpus_Decode(OpusDecInst* inst, const uint8_t* encoded,
                      size_t encoded_bytes, int16_t* decoded,
                      int16_t* audio_type) {
  int decoded_samples;

  if (encoded_bytes == 0) {
    *audio_type = DetermineAudioType(inst, encoded_bytes);
    decoded_samples = WebRtcOpus_DecodePlc(inst, decoded, 1);
  } else {
    decoded_samples = DecodeNative(inst,
                                   encoded,
                                   encoded_bytes,
                                   kWebRtcOpusMaxFrameSizePerChannel,
                                   decoded,
                                   audio_type,
                                   0);
  }
  if (decoded_samples < 0) {
    return -1;
  }

  /* Update decoded sample memory, to be used by the PLC in case of losses. */
  inst->prev_decoded_samples = decoded_samples;

  return decoded_samples;
}

```---



## 网络拥塞控制

<style type="text/css">
  .reveal h2{
    text-align: left;
    font-size: 25px;
  }  
</style>


### 基于丢包

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>

* 丢包率与带宽有时没有直接关系，导致误判。即丢包很多时候并非由于网络带宽不够用；
* 历史估计未来，缺乏预判性。假定丢包是因为带宽不够用导致，等感知到丢包，说明网络已经拥塞，这将导致视频卡顿等问题；
* 通过感知到丢包到应用层才去策略，需要一定的时间，这段时间将进一步导致拥塞。这在对延迟要求高的场景下，效果不理想。

---

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/loss_estimate.png)

---

### 基于延时

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>


接收端采用 Google Congestion Control 算法，简称GCC。
[GCC](https://tools.ietf.org/html/draft-alvestrand-rtcweb-congestion-02)


![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_estimate.png)

---

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 25px;
  }  
</style>

GCC 算法处理流程

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_process.png)



---

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>


![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/gcc_ts_delta.png)

* d(i) = t(i) – t(i-1) – (T(i) – T(i-1))
* dL(i) = L(i) - L(i-1)
* 带宽估算模型： d(i) = dL(i) / C(i) + m(i) + v(i)    

---

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>

* dL(i)表示帧i和帧i-1的长度之差，
* C(i)表示信道传输速率，
* m(i)表示帧i的网络排队时延，
* v(i)表示测量噪声，其协方差为R。
其中[1/C(i) m(i)]是我们要求的目标值，即信道传输速率和网络排队时延。

---

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>

* 构建卡尔曼滤波模型, 估算出 m(i)
* 网络状态检测用当前网络延迟 m(i) 和阈值(动态更新) 进行比较，判断出overuse，underuse 和 normal 三种网络状态之一. 
* 码率控制模块维护三种码率变化趋势(Hold, Increase, Decrease),估算真正的码率 Ar. 
* 当码率变化趋势为 Increase 时，当前码率为上次码率乘上系数 1.05；当码率变化趋势为 Decrease，当前码率为过去 500ms 内的最大接收码率乘上系数 0.85。当码率变化趋势为 Hold 时，当前码率保持不变。

```
   State ---->  | Hold      |Increase    |Decrease
   Signal-----------------------------------------
     v          |           |            |
   Over-use     | Decrease  |Decrease    |
   -----------------------------------------------
   Normal       | Increase  |            |Hold
   -----------------------------------------------
   Under-use    |           |Hold        |Hold
   -----------------------------------------------

```



---

卡夫曼模型代码

```
void OveruseEstimator::Update(int64_t t_delta,
                              double ts_delta,
                              int size_delta,
                              BandwidthUsage current_hypothesis,
                              int64_t now_ms) {
  const double min_frame_period = UpdateMinFramePeriod(ts_delta);
  const double t_ts_delta = t_delta - ts_delta;
  BWE_TEST_LOGGING_PLOT(1, "dm_ms", now_ms, t_ts_delta);
  double fs_delta = size_delta;

  ++num_of_deltas_;
  if (num_of_deltas_ > kDeltaCounterMax) {
    num_of_deltas_ = kDeltaCounterMax;
  }

  // Update the Kalman filter.
  E_[0][0] += process_noise_[0];
  E_[1][1] += process_noise_[1];

  if ((current_hypothesis == BandwidthUsage::kBwOverusing &&
       offset_ < prev_offset_) ||
      (current_hypothesis == BandwidthUsage::kBwUnderusing &&
       offset_ > prev_offset_)) {
    E_[1][1] += 10 * process_noise_[1];
  }

  const double h[2] = {fs_delta, 1.0};
  const double Eh[2] = {E_[0][0]*h[0] + E_[0][1]*h[1],
                        E_[1][0]*h[0] + E_[1][1]*h[1]};

  BWE_TEST_LOGGING_PLOT(1, "d_ms", now_ms, slope_ * h[0] - offset_);

  const double residual = t_ts_delta - slope_*h[0] - offset_;

  const bool in_stable_state =
      (current_hypothesis == BandwidthUsage::kBwNormal);
  const double max_residual = 3.0 * sqrt(var_noise_);
  // We try to filter out very late frames. For instance periodic key
  // frames doesn't fit the Gaussian model well.
  if (fabs(residual) < max_residual) {
    UpdateNoiseEstimate(residual, min_frame_period, in_stable_state);
  } else {
    UpdateNoiseEstimate(residual < 0 ? -max_residual : max_residual,
                        min_frame_period, in_stable_state);
  }

  const double denom = var_noise_ + h[0]*Eh[0] + h[1]*Eh[1];

  const double K[2] = {Eh[0] / denom,
                       Eh[1] / denom};

  const double IKh[2][2] = {{1.0 - K[0]*h[0], -K[0]*h[1]},
                            {-K[1]*h[0], 1.0 - K[1]*h[1]}};
  const double e00 = E_[0][0];
  const double e01 = E_[0][1];

  // Update state.
  E_[0][0] = e00 * IKh[0][0] + E_[1][0] * IKh[0][1];
  E_[0][1] = e01 * IKh[0][0] + E_[1][1] * IKh[0][1];
  E_[1][0] = e00 * IKh[1][0] + E_[1][0] * IKh[1][1];
  E_[1][1] = e01 * IKh[1][0] + E_[1][1] * IKh[1][1];

  // The covariance matrix must be positive semi-definite.
  bool positive_semi_definite = E_[0][0] + E_[1][1] >= 0 &&
      E_[0][0] * E_[1][1] - E_[0][1] * E_[1][0] >= 0 && E_[0][0] >= 0;
  assert(positive_semi_definite);
  if (!positive_semi_definite) {
    LOG(LS_ERROR) << "The over-use estimator's covariance matrix is no longer "
                     "semi-definite.";
  }

  slope_ = slope_ + K[0] * residual;
  prev_offset_ = offset_;
  offset_ = offset_ + K[1] * residual;

  BWE_TEST_LOGGING_PLOT(1, "kc", now_ms, K[0]);
  BWE_TEST_LOGGING_PLOT(1, "km", now_ms, K[1]);
  BWE_TEST_LOGGING_PLOT(1, "slope_1/bps", now_ms, slope_);
  BWE_TEST_LOGGING_PLOT(1, "var_noise", now_ms, var_noise_);
}

```


---

## 关键帧请求

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
  .reveal p{
    text-align: left;
    font-size: 18px;
  }
</style>

关键帧也叫做帧内刷新帧，简称IDR帧。对视频来说，IDR帧的解码无需参考之前的帧，因此在丢包严重时可以通过发送关键帧请求进行画面的恢复。关键帧的请求方式分为：RTCP FIR反馈（Full intra frame request; RTCP PLI反馈（Picture Loss Indictor）

UPRTC 通过关键帧请求实现秒开的功能。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/I_frame.png)


---

## 抖动、乱序

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
  .reveal p{
    text-align: left;
    font-size: 18px;
  } 
</style>

抖动, 乱序主要依赖于 jitter buffer 修正, 但同时也会与其他的 qos 策略紧密相连。当有些包很久（一般大于500ms）都没收到，就认为它来不了，直接将它们丢弃；有些包短时间（小于50ms）没来，则向发送端发送重传请求，请求发送端再发一次该包，试图能补上这些包(nack). 丢帧则会请求 fir. 调整 jitter buffer 将牺牲延时为代价, 需要在延时和丢包、抖动之间做出平衡。

![qos_qoe](https://upmedia.b0.upaiyun.com/assets/qos/jitter_buffer.jpg)

---



## 结语

<style type="text/css">
  .reveal {
    text-align: left;
    font-size: 18px;
  }  
</style>

QOS保障技术，不同的技术对于不同的网络状况效果不同，比如 ARQ 重传在小丢包和较低网络延迟的网络情况下能够达到较理想的效果, 音视频 QOS 的关键问题是如何能够整合这些技术，使得各个技术能够发挥到最佳效果，从而提高音视频的体验。

在互联网上为实时交互式音视频应用提供 QoS 保证仍是一项挑战，需要音视频编码器、传输、预处理等多模块的协作配合，或利用现有网络协议和设备的支持，才能提供给客户更多的选择和服务保证。

---

<!--<style type="text/css">
  .reveal p {
    text-align: center;
    font-size: 40px;
  }  
</style>-->

谢谢聆听!
